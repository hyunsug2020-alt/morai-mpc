"""Read-only LiDAR, map and decision drawing in the vehicle frame."""
import math
import textwrap

import cv2
import numpy as np

from ego_body import EGO_CENTRE_M, EGO_FRONT_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, EGO_REAR_M


import os

# Black/red "spider HUD" palette (BGR); STUDENT_PLAN_THEME=classic restores the old one.
SPIDER = os.environ.get("STUDENT_PLAN_THEME", "spider") != "classic"
if SPIDER:
    ROUTE = (150, 210, 255)        # warm white-gold global route
    CHECKED = (70, 60, 255)        # hot red control path
    MODEL = (200, 120, 255)
    RISK = (40, 40, 255)
    MAP = (50, 40, 120)            # dim red corridor edge (control path is the bright one)
    BG, GRID, GRID_TEXT = (12, 8, 14), (28, 18, 60), (90, 80, 170)
    FILL_CORRIDOR, FILL_RIBBON = (24, 14, 70), (30, 20, 140)
    LIDAR, OBJECT, PERSON, BODY = (120, 110, 255), (60, 170, 255), (255, 210, 120), (235, 235, 255)
    PANEL, HEADER, TEXT = (16, 10, 20), (10, 6, 14), (200, 200, 230)
else:
    ROUTE = (30, 205, 255)
    CHECKED = (255, 230, 60)
    MODEL = (210, 110, 240)
    RISK = (75, 75, 255)
    MAP = (155, 155, 115)
    BG, GRID, GRID_TEXT = (24, 29, 35), (47, 55, 62), (130, 143, 150)
    FILL_CORRIDOR, FILL_RIBBON = (53, 65, 60), (105, 90, 35)
    LIDAR, OBJECT, PERSON, BODY = (70, 165, 225), (210, 166, 108), (75, 180, 255), (235, 240, 245)
    PANEL, HEADER, TEXT = (30, 36, 43), (19, 24, 30), (200, 209, 215)


def hud_brackets(img, x0, y0, x1, y1, color, size=14, thickness=2):
    """Iron-Man style corner brackets."""
    for (x, y, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(img, (x, y), (x + dx * size, y), color, thickness, cv2.LINE_AA)
        cv2.line(img, (x, y), (x, y + dy * size), color, thickness, cv2.LINE_AA)


def draw_spider_car(view, pixels, color_body=(40, 30, 200), color_line=(235, 235, 255)):
    """Ego car (real 4.635 x 1.892 m footprint) in red with a black spider emblem.
    The pose is the rear axle: the body is centred EGO_CENTRE_M ahead of it (ego_body)."""
    hl, hw = EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
    to_pixels = pixels

    def pixels(points):
        return to_pixels([(x + EGO_CENTRE_M, y) for x, y in points])

    outline = [(hl - .35, hw), (hl, hw - .35), (hl, -hw + .35), (hl - .35, -hw), (-hl, -hw), (-hl, hw)]
    body = pixels(outline)
    if len(body) < 3:
        return
    cv2.fillPoly(view, [body], color_body, cv2.LINE_AA)
    cv2.polylines(view, [body], True, color_line, 2, cv2.LINE_AA)
    glass = pixels([(hl - .6, hw - .25), (hl - .6, -hw + .25), (hl - 1.3, -hw + .3), (hl - 1.3, hw - .3)])
    cv2.fillPoly(view, [glass], (20, 10, 30), cv2.LINE_AA)
    (cx, cy), = pixels([(-.4, 0.)])
    unit = max(2, int(abs(pixels([(0., 0.)])[0, 0] - pixels([(0., .3)])[0, 0])))
    for side in (-1, 1):            # eight legs
        for k, (ox, oy) in enumerate(((-1.4, -2.4), (-.5, -2.7), (.5, -2.7), (1.4, -2.4))):
            knee = (cx + side * int(oy * unit * .45), cy + int(ox * unit * .9) - unit // 2)
            foot = (cx + side * int(oy * unit * .8), cy + int(ox * unit * 1.6))
            cv2.line(view, (cx, cy), knee, (10, 10, 10), 1, cv2.LINE_AA)
            cv2.line(view, knee, foot, (10, 10, 10), 1, cv2.LINE_AA)
    cv2.ellipse(view, (cx, cy + unit // 2), (max(2, unit // 2), unit), 0, 0, 360, (10, 10, 10), -1, cv2.LINE_AA)
    cv2.circle(view, (cx, cy - unit), max(2, unit // 3), (10, 10, 10), -1, cv2.LINE_AA)


def local_xy(world, ego):
    yaw = math.radians(float(ego['heading_deg']))
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return (np.asarray(world, dtype=float)-[ego['x'], ego['y']]) @ rotation


def object_polygon(obj, ego):
    """Draw the same world box axis as SAT, independent of its motion direction."""
    length = max(.3, float(obj[6] if len(obj) > 6 else obj[2]))
    width = max(.3, float(obj[7] if len(obj) > 7 else obj[2]))
    yaw = math.radians(float(obj[8] if len(obj) > 8 else 0.))
    axis = np.array([math.cos(yaw), math.sin(yaw)])
    normal = np.array([-axis[1], axis[0]])
    world = np.asarray(obj[:2])+np.array([
        a*axis*length/2.+b*normal*width/2.
        for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
    return local_xy(world, ego)


def render_plan_view(canvas, origin, size, *, result, record, lidar_points=None,
                     lidar_age_s=None, links=(), corridor=None, objects=(), ego=None):
    """Render observed geometry and the already selected path; never plan or publish."""
    x0, y0 = origin
    width, height = size
    frame = canvas[y0:y0+height, x0:x0+width]
    frame[:] = BG
    panel_width = min(228, width//3)
    map_width = width-panel_width
    top, bottom = 58, height-39
    scale = (bottom-top)/60.
    centre = map_width//2
    view = frame[top:bottom+23, :map_width]

    def pixels(values):
        values = np.asarray(values, dtype=float)
        if values.size == 0:
            return np.empty((0, 2), dtype=np.int32)
        values = values.reshape(-1, values.shape[-1])[:, :2]
        values = values[np.isfinite(values).all(axis=1)]
        return np.rint(np.column_stack((centre-values[:, 1]*scale,
                                       bottom-top-values[:, 0]*scale))).astype(np.int32)

    def polyline(values, color, thickness=1, closed=False):
        p = pixels(values)
        if len(p) > 1:
            cv2.polylines(view, [p], closed, color, thickness, cv2.LINE_AA)

    for forward in range(0, 61, 10):
        row = int(bottom-top-forward*scale)
        cv2.line(view, (0, row), (map_width, row), GRID, 1)
        cv2.putText(view, f'{forward}m', (8, max(13, row-4)), 0, .38,
                    GRID_TEXT, 1)
    for lateral in range(-30, 31, 5):
        col = int(centre-lateral*scale)
        if 0 < col < map_width:
            cv2.line(view, (col, 0), (col, len(view)), GRID, 1)
    if SPIDER:
        # Range rings and a radial web around the car (HUD look).
        car = tuple(pixels([(0., 0.)])[0])
        for metres in (10, 20, 30, 40, 50):
            r = int(metres*scale)
            cv2.ellipse(view, car, (r, r), 0, 180, 360, (40, 26, 110) if metres % 20 else (55, 35, 150), 1, cv2.LINE_AA)
        for angle in range(200, 341, 20):
            a = math.radians(angle)
            end = (int(car[0]+math.cos(a)*55*scale), int(car[1]+math.sin(a)*55*scale))
            cv2.line(view, car, end, (34, 22, 80), 1, cv2.LINE_AA)

    if corridor is not None and len(corridor.route) > 1:
        left = corridor.route+corridor.normal*np.asarray(corridor.left)[:, None]
        right = corridor.route+corridor.normal*np.asarray(corridor.right)[:, None]
        area = pixels(np.vstack((left, right[::-1])))
        overlay = view.copy()
        if len(area) >= 3:
            cv2.fillPoly(overlay, [area], FILL_CORRIDOR)
            cv2.addWeighted(overlay, .65, view, .35, 0, view)
        polyline(left, MAP, 2)
        polyline(right, MAP, 2)
    for link in links:
        polyline(link, (92, 101, 112))

    tunnel = record.get('tunnel') or {}
    clear = tunnel.get('clear_distance_m')
    if clear is None:
        clear = (record.get('highway') or {}).get('clear_distance_m')
    if clear is None:
        clear = record.get('static_clear_distance_m')
    if clear is None:
        clear = (record.get('lidar') or {}).get('path_block_m')
    stopped = bool(record.get('motion_state') == 'STOP' or record.get('emergency_stop')
                   or record.get('published_cap_kmh') == 0)
    if result:
        path = np.asarray(result['path'], dtype=float)[:, :2]
        polyline(result['local_route'], ROUTE, 2)
        if record.get('action') != 'LIDAR':
            polyline(result['raw_path'], MODEL, 1)
        if len(path) > 1:
            # Body ribbon shows the actual selected trajectory, including a
            # deliberate offset from the global route during a bypass.
            tangent = np.gradient(path, axis=0)
            tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
            tangent[0] = (1., 0.)
            normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            horizon = min(60., float(clear)) if clear is not None and math.isfinite(float(clear)) else 60.
            selected = arc <= max(0., horizon)
            if selected.sum() >= 2:
                p = path[selected]
                n = normal[selected]
                ribbon = pixels(np.vstack((p+.946*n, (p-.946*n)[::-1])))
                overlay = view.copy()
                cv2.fillPoly(overlay, [ribbon], FILL_RIBBON)
                cv2.addWeighted(overlay, .4, view, .6, 0, view)
                if SPIDER:
                    polyline(p, (40, 30, 150), 9)        # glow
                polyline(p, CHECKED, 3)
            polyline(path[arc > max(0., horizon)], (105, 115, 120), 1)

    points = pixels(lidar_points if lidar_points is not None else ())
    if len(points):
        visible = ((points[:, 0] >= 0) & (points[:, 0] < map_width) &
                   (points[:, 1] >= 0) & (points[:, 1] < len(view)))
        p = points[visible]
        view[p[:, 1], p[:, 0]] = LIDAR

    risks = record.get('traffic_risks') or ()
    risky_ids = {str(r.get('object_id')) for r in risks}
    labels = 0
    for obj in objects if ego else ():
        if len(obj) < 5:
            continue
        uid = str(obj[11] if len(obj) > 11 else obj[3])
        color = RISK if uid in risky_ids else (PERSON if obj[4] == 'pedestrian' else OBJECT)
        polyline(object_polygon(obj, ego), color, 2, True)
        p = pixels(local_xy(np.asarray(obj[:2])[None], ego))
        position = local_xy(np.asarray(obj[:2])[None], ego)[0]
        important = uid in risky_ids
        nearby = 2. <= position[0] <= 35. and abs(position[1]) <= 12. and float(obj[2]) >= .7
        if (important or nearby and labels < 8) and len(p) and 0 <= p[0, 0] < map_width and 0 <= p[0, 1] < len(view):
            kind = {'npc': 'car', 'pedestrian': 'person', 'obstacle': 'object'}.get(obj[4], obj[4])
            cv2.putText(view, f'{kind} #{uid[-8:]}', tuple(p[0]+[8, -6]),
                        0, .33, color, 1, cv2.LINE_AA)
            labels += 1
        valid_motion = len(obj) > 10 and (obj[3] != 'lidar_car' or len(obj) > 14 and obj[14])
        if valid_motion and np.isfinite(obj[9:11]).all() and math.hypot(*obj[9:11]) > .3:
            ends = pixels(local_xy(np.array([obj[:2], np.asarray(obj[:2])+np.asarray(obj[9:11])]), ego))
            if len(ends) == 2:
                cv2.arrowedLine(view, tuple(ends[0]), tuple(ends[1]), color, 1, cv2.LINE_AA, tipLength=.2)
    for risk in risks[:3] if ego else ():
        if not all(key in risk for key in ('object_xy', 'ego_xy')):
            continue
        positions = pixels(local_xy([risk['object_xy'], risk['ego_xy']], ego))
        if len(positions) != 2:
            continue
        a, b = positions
        cv2.line(view, tuple(a), tuple(b), RISK, 1, cv2.LINE_AA)
        cv2.circle(view, tuple(a), 9, RISK, 2, cv2.LINE_AA)
        cv2.putText(view, f"risk +{float(risk.get('when', 0.)):.1f}s", tuple(a+[10, -8]),
                    0, .36, RISK, 1, cv2.LINE_AA)
    stop_line = record.get('stop_line') or {}
    if ego and stop_line.get('line_xy'):
        p = local_xy(stop_line['line_xy'], ego)
        polyline([p+[0., -2.], p+[0., 2.]], RISK if stop_line.get('stop') else (110, 200, 135), 3)
    # The ego body uses its physical dimensions rather than a symbolic dot.
    if SPIDER:
        draw_spider_car(view, pixels)
    else:
        polyline([(EGO_FRONT_M, .946), (EGO_FRONT_M, -.946), (-EGO_REAR_M, -.946), (-EGO_REAR_M, .946)],
                 BODY, 2, True)
        a, b = pixels([(0., 0.), (3., 0.)])
        cv2.arrowedLine(view, tuple(a), tuple(b), BODY, 2, cv2.LINE_AA, tipLength=.25)

    cv2.rectangle(frame, (0, 0), (width, top-1), HEADER, -1)
    if SPIDER:
        cv2.line(frame, (0, top-1), (width, top-1), (40, 30, 200), 2, cv2.LINE_AA)
        hud_brackets(frame, 4, 4, width-5, height-5, (50, 40, 230))
    cap = record.get('published_cap_kmh')
    cap_text = f'{float(cap):.1f} km/h' if isinstance(cap, (float, int)) else '--'
    state = 'STOP' if stopped else 'DRIVE' if result else 'WAITING'
    state_color = RISK if stopped else ((120, 255, 200) if SPIDER else (135, 220, 120))
    cv2.putText(frame, f'{state}  |  target {cap_text}', (12, 24), 0, .58, state_color, 2, cv2.LINE_AA)
    mode = str(record.get('mode') or 'LiDAR + HDMAP')
    cv2.putText(frame, mode, (12, 45), 0, .42, TEXT, 1, cv2.LINE_AA)
    cv2.rectangle(frame, (map_width, top), (width, height), PANEL, -1)
    if SPIDER:
        cv2.line(frame, (map_width, top), (map_width, height), (40, 30, 160), 1, cv2.LINE_AA)
        hud_brackets(frame, map_width+4, top+4, width-10, height-34, (40, 30, 180), 10, 1)
    tx, ty = map_width+12, top+23

    def text(value, color=TEXT, gap=19):
        nonlocal ty
        for line in textwrap.wrap(str(value), width=max(16, int((panel_width-20)/6.6))):
            if ty >= height-30:
                return
            cv2.putText(frame, line, (tx, ty), 0, .37, color, 1, cv2.LINE_AA)
            ty += gap

    text('ACTUAL DECISION', CHECKED)
    text(record.get('stop_reason') or record.get('reason') or 'Waiting for a checked path', state_color)
    ty += 10
    text(f"ego {float(record.get('speed_kmh') or 0.):.1f} km/h")
    text(f"LiDAR {len(points)} pts / {float(lidar_age_s):.2f}s" if lidar_age_s is not None else 'LiDAR unavailable')
    text(f"decision {float(record.get('decision_hz') or 0.):.1f} Hz")
    if clear is not None:
        text(f'path clear {float(clear):.1f}m', CHECKED)
    for key, distance in (tunnel.get('clearance_components_m') or {}).items():
        limiting = key == tunnel.get('limiting_source')
        text(f'{key}: {float(distance):.1f}m' + (' [min]' if limiting else ''),
             (100, 195, 235) if limiting else (170, 185, 190))
    if stop_line:
        text(f"stop line: {stop_line.get('state', '--')}")
        text(f"bumper {float(stop_line.get('distance_m') or 0.):.2f}m")
    if risks:
        text(f'timed collision risks: {len(risks)}', RISK)
    guidance = tunnel.get('model_guidance')
    if guidance:
        text('best_v5: ' + ('ACCEPTED' if guidance.get('accepted') and guidance.get('source') == 'model'
                          else 'LiDAR FALLBACK'), CHECKED)
        text(guidance.get('reason', 'model candidate unavailable'))
    ty += 8
    for label, color in [('HDMAP corridor', MAP), ('Global route', ROUTE),
                         ('Actual control path', CHECKED), ('Model proposal', MODEL),
                         ('LiDAR returns', LIDAR), ('Collision prediction', RISK)]:
        text(label, color, 18)
    cv2.putText(frame, 'BODY footprint  |  10m grid  |  forward UP / left LEFT',
                (10, height-10), 0, .36, GRID_TEXT if SPIDER else (160, 174, 187), 1, cv2.LINE_AA)
