"""Semantic housing selection, lamp-slot HSV classification, and temporal decisions."""
import math

import cv2
import numpy as np
from .permissions import policy_allows, LEGACY


def roi_pixels(config, shape):
    height, width = shape[:2]
    a, b, c, d = config["roi"]
    return [round(a * width), round(b * height), round(c * width), round(d * height)]


def hue_mask(hsv, ranges, config):
    h, s, v = cv2.split(hsv)
    allowed = np.zeros(h.shape, bool)
    for lo, hi in zip(ranges[::2], ranges[1::2]):
        allowed |= (h >= lo) & (h <= hi)
    return (allowed & (s >= config["s_min"]) & (v >= config["v_min"])).astype(np.uint8) * 255


def requires_stop(state, maneuver, policy=LEGACY):
    if policy != LEGACY:
        return not policy_allows(state, policy)
    if state in ("UNKNOWN", "RED", "YELLOW"):
        return True
    if state in ("LEFT", "RED_LEFT"):
        return maneuver != "left"
    if state == "GREEN":
        return maneuver == "left"
    return state != "GREEN_LEFT"


def mapped_lamp_bounds(expected, x, width, layout):
    """Correct sub-lamp mask jitter only when map and semantic edges agree.

    At 35 m a complete four-lamp housing is just 11 pixels wide. One pixel
    of sky in its semantic box can move straight green into the arrow slot.
    The map never selects a housing without a matching semantic component.
    """
    if expected is None or layout not in ("three", "four"):
        return None
    left, right = float(expected[0])-x, float(expected[2])-x
    count = 4 if layout == "four" else 3
    span = right-left
    tolerance = span/count/2
    # A semantic box may include a few pixels of sky outside the mapped
    # housing. Extra background must not move its lamp divisions. Missing
    # mapped housing is limited to half a lamp; excess padding to half a housing.
    if (not math.isfinite(left) or not math.isfinite(right) or span < count or
            left < -tolerance or right > width+tolerance or
            left > span/2 or width-right > span/2):
        return None
    return left, right


def classify_housing(crop, config, lamp_bounds=None):
    """Positions distinguish the arrow's green from the rightmost straight-green lamp.

    Four-lamp default is RED | YELLOW | LEFT | GREEN, as calibrated for this camera.
    This detects a lit arrow slot; it does not infer arrow direction from HSV alone.
    """
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    masks = {name: hue_mask(hsv, config[name + "_h"], config) for name in ("red", "yellow", "green")}
    order = ["red", "yellow", "left", "green"] if config["layout"] == "four" else ["red", "yellow", "green"]
    h, w = crop.shape[:2]
    left, right = lamp_bounds if lamp_bounds is not None else (0., float(w))
    step = (right-left)/len(order)
    y1, y2 = round(h * .10), max(round(h * .90), round(h * .10) + 1)
    assigned = {name: np.zeros((h, w), np.uint8) for name in order}
    for color, binary in masks.items():
        interior = binary.copy()
        interior[:y1] = 0
        interior[y2:] = 0
        n, labels, boxes, centers = cv2.connectedComponentsWithStats(interior, 8)
        for component in range(1, n):
            if boxes[component, cv2.CC_STAT_AREA] < config["min_pixels"]:
                continue
            position = (centers[component, 0]-left)/step
            if not 0. <= position < len(order):
                continue
            slot = order[int(position)]
            expected = "green" if slot == "left" else slot
            if expected == color:
                assigned[slot][labels == component] = 255
    # Assign each connected glow to one lamp before measuring slots. A bright straight-green
    # circle often spills over the boundary into the arrow slot at this resolution.
    slots = {}
    for i, name in enumerate(order):
        # Trim housing edges; use nearest slot boundaries so neighbouring greens cannot mix.
        x1, x2 = max(0, min(w, round(left+i*step))), max(0, min(w, round(left+(i+1)*step)))
        patch = assigned[name][y1:y2, x1:x2]
        count = int(np.count_nonzero(patch))
        n, _, boxes, _ = cv2.connectedComponentsWithStats(patch, 8)
        largest = int(boxes[1:, cv2.CC_STAT_AREA].max()) if n > 1 else 0
        minimum = max(config["min_pixels"], math.ceil(patch.size * config["min_fraction"]))
        slots[name] = {"pixels": count, "largest_blob": largest, "minimum": minimum,
                       "on": largest >= minimum, "box": [x1, y1, x2 - x1, y2 - y1]}
    red, yellow, green = (slots[name]["on"] for name in ("red", "yellow", "green"))
    left = slots.get("left", {}).get("on", False)
    if yellow:
        state = "YELLOW"  # Any detected yellow requests stopping immediately.
    elif red and green:
        state = "UNKNOWN"
    elif red:
        state = "RED_LEFT" if left else "RED"
    elif green:
        state = "GREEN_LEFT" if left else "GREEN"
    elif left:
        state = "LEFT"
    else:
        state = "UNKNOWN"
    return {"raw_state": state, "slots": slots, "left_on": left,
            "straight_color": "yellow" if yellow else "red" if red else "green" if green else "unknown",
            "raw_stop_required": requires_stop(state, config["maneuver"], config.get("permission_policy", LEGACY))}, masks


def analyze_frame(image, semantic_mask, config, target=None):
    x1, y1, x2, y2 = roi_pixels(config, image.shape)
    full = (semantic_mask > 0).astype(np.uint8)
    _, _, full_boxes, _ = cv2.connectedComponentsWithStats(full, 8)
    semantic_boxes = [box.tolist() for box in full_boxes[1:] if box[4] >= 3]
    binary = (semantic_mask[y1:y2, x1:x2] > 0).astype(np.uint8)
    roi_count = int(np.count_nonzero(binary))
    # Closing fills tiny breaks in a housing, without letting HSV find unrelated colored pixels.
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, labels, boxes, centroids = cv2.connectedComponentsWithStats(binary, 8)
    candidates = []
    clipped = False
    window = expected = None
    if target is not None:
        scale = np.array([image.shape[1], image.shape[0]]*2, dtype=float)
        if target.get("window") is not None:
            window = np.asarray(target["window"], dtype=float)*scale
            expected = np.asarray(target["expected_box"], dtype=float)*scale
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in boxes[i])
        lamp_bounds = mapped_lamp_bounds(expected, x+x1, w, config["layout"])
        # A mapped three-lamp signal is only 7 x 3 pixels on the first
        # approach. Require the same semantic component and mapped lamp
        # edges, rather than rejecting its real green for being sub-8 x 4.
        small_mapped = bool(config["layout"] == "three" and lamp_bounds is not None
                            and expected[2]-expected[0] <= 16.)
        min_area, min_width, min_height = ((3, 3, 2) if small_mapped else
                                         (config["min_semantic_area"], 8, 4))
        if area < min_area or w < min_width or h < min_height or not 1.6 <= w / h <= 7:
            continue
        if x <= 0 or x + w >= x2 - x1:
            clipped = True
            continue  # A clipped housing would shift lamp slots and corrupt arrow identification.
        center = centroids[i]
        absolute_center = center + [x1, y1]
        if target is not None and (window is None or not
                (window[0] <= absolute_center[0] <= window[2] and window[1] <= absolute_center[1] <= window[3])):
            continue
        if small_mapped:
            # Semantic edges can omit the last column of a lit lamp. Expand
            # only to the already matched map edges and one vertical pixel;
            # HSV still needs the configured current-frame connected blob.
            right, bottom = x+w, y+h
            x = max(0, min(x, math.floor(expected[0])-x1))
            right = min(x2-x1, max(right, math.ceil(expected[2])-x1))
            y, bottom = max(0, y-1), min(y2-y1, bottom+1)
            w, h = right-x, bottom-y
            lamp_bounds = mapped_lamp_bounds(expected, x+x1, w, config["layout"])
        vertical_clipped = y <= 0 or y + h >= y2 - y1
        observation, masks = classify_housing(image[y+y1:y+y1+h, x+x1:x+x1+w], config, lamp_bounds)
        # Vertical clipping leaves the horizontal lamp slots intact. Permission
        # still requires complete lit blobs, away from the clipped image edge.
        if vertical_clipped and any(np.any(m[0]) if y <= 0 else False for m in masks.values()):
            observation["raw_state"] = "UNKNOWN"
        if vertical_clipped and y + h >= y2-y1 and any(np.any(m[-1]) for m in masks.values()):
            observation["raw_state"] = "UNKNOWN"
        distance = ((center[0] - (x2 - x1) / 2) / (x2 - x1)) ** 2 + ((center[1] - (y2 - y1) / 2) / (y2 - y1)) ** 2
        candidates.append({"box": [x + x1, y + y1, w, h], "semantic_area": area, "distance": float(distance),
                           "lamp_bounds": lamp_bounds,
                           "vertical_clipped": vertical_clipped, "lamp_state": observation["raw_state"],
                           "map_distance": float(np.sum(((absolute_center-(expected[:2]+expected[2:])/2) /
                               np.maximum(8., expected[2:]-expected[:2]))**2)) if expected is not None else 0.})
    result = {"raw_state": "UNKNOWN", "raw_stop_required": True, "target_box": None,
              "reason": "ROI 안 신호등 영역 없음", "candidates": candidates, "roi_pixels": [x1, y1, x2, y2],
              "semantic_boxes": semantic_boxes, "semantic_pixels": int(np.count_nonzero(full)),
              "roi_semantic_pixels": roi_count,
              "slots": {}, "left_on": False, "straight_color": "unknown"}
    if target is not None:
        result.update(traffic_light_id=target["traffic_light_id"],
                      mapped_search_box=window.tolist() if window is not None else None,
                      mapped_expected_box=expected.tolist() if expected is not None else None)
    if not candidates:
        if target is not None:
            result.update(detection_stage="mapped_signal_missing",
                          reason="지도상 해당 신호등 위치에 시맨틱 신호등 미검출" if window is not None
                          else "지도상 해당 신호등이 카메라 시야 밖에 있음")
        elif roi_count == 0:
            result["detection_stage"] = "outside_roi" if result["semantic_pixels"] else "semantic_missing"
            result["reason"] = ("시맨틱 신호등 후보가 설정 ROI 밖에 있음 · ROI 안 검출 0" if result["semantic_pixels"]
                                else "전체 영상에서 시맨틱 신호등 미검출")
        elif clipped:
            result.update(detection_stage="clipped", reason="신호등 영역이 ROI 경계에서 잘려 판정 보류")
        else:
            result.update(detection_stage="shape_rejected", reason="ROI 안 시맨틱 영역이 신호등 크기·형태 조건 미달")
        return result, None, {}
    # Color never chooses the housing: a green reflection must not displace the
    # mapped signal, even when its lit lamps are temporarily unrecognized.
    candidates.sort(key=lambda c: (c["map_distance"], -c["semantic_area"], c["distance"]))
    target = candidates[0]
    x, y, w, h = target["box"]
    crop = image[y:y+h, x:x+w].copy()
    classified, masks = classify_housing(crop, config, target["lamp_bounds"])
    if target["lamp_state"] == "UNKNOWN":
        classified.update(raw_state="UNKNOWN", raw_stop_required=True)
    result.update(classified, detection_stage="hsv", target_box=target["box"], semantic_area=target["semantic_area"],
                  lamp_bounds=target["lamp_bounds"],
                  vertical_clipped=target["vertical_clipped"],
                  reason="시맨틱 신호등 영역 안 HSV · 등 위치 판정")
    return result, crop, masks


def box_iou(first, second):
    if first is None or second is None:
        return 0.0
    x, y, w, h = first
    a, b, c, d = second
    intersection = max(0, min(x+w, a+c)-max(x, a)) * max(0, min(y+h, b+d)-max(y, b))
    return intersection / max(1, w*h + c*d - intersection)


def same_moving_housing(first, second):
    """An approaching housing moves up rapidly and grows between 5 Hz frames."""
    if first is None or second is None:
        return False
    if box_iou(first, second) >= .4:
        return True
    x, y, w, h = first
    a, b, c, d = second
    if not (.5 <= c/max(w, 1) <= 2 and .5 <= d/max(h, 1) <= 2):
        return False
    displacement = math.hypot(a+c/2-x-w/2, b+d/2-y-h/2)
    return displacement <= .75*max(w, c)


class StableSignal:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pending, self.stable, self.count, self.box = "UNKNOWN", "UNKNOWN", 0, None

    def update(self, observation, config):
        box, raw = observation["target_box"], observation["raw_state"]
        if box is None or (self.box is not None and not same_moving_housing(self.box, box)):
            self.reset()
        if raw == self.pending and box is not None:
            self.count += 1
        else:
            self.pending, self.count = raw, 1
        self.box = box
        if self.count >= config["stable_frames"]:
            self.stable = raw
        # Stop transitions are immediate; permission requires matching stable frames.
        policy = config.get("permission_policy", LEGACY)
        stop = (requires_stop(raw, config["maneuver"], policy) or requires_stop(self.stable, config["maneuver"], policy)
                or self.count < config["stable_frames"])
        return dict(observation, state=self.stable, stable_count=self.count, stop_required=stop,
                    permission_stable=self.count >= config['stable_frames'],
                    decision="STOP" if stop else "ALLOW", maneuver=config["maneuver"],
                    stop_on_yellow=True)
