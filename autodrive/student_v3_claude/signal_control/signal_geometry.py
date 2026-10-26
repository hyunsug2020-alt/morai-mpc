"""Route geometry shared by the controller and dashboard without ROS/OpenCV."""
# Ioniq5 actor/GPS origin is behind the bounding-box centre.  Half the
# vehicle length alone is not the distance from that origin to the bumper.
# Read-only simulator geometry: centre +1.5264 m, half length 2.3175 m.
# Fixed vehicle geometry; simulator truth is not a driving input.
from ego_body import (EGO_HALF_LENGTH_M as VEHICLE_HALF_LENGTH_M,
                      EGO_CENTRE_M as VEHICLE_CENTRE_FORWARD_M,
                      EGO_FRONT_M as FRONT_BUMPER_M)


def projected_route_station(route, stations, x, y, nearest_idx):
    index = min(max(0, int(nearest_idx)), len(route)-1)
    best = (float("inf"), float(stations[index]))
    for i in range(max(0, index-2), min(len(route)-1, index+3)):
        ax, ay = float(route[i][0]), float(route[i][1])
        dx, dy = float(route[i+1][0])-ax, float(route[i+1][1])-ay
        length2 = dx*dx+dy*dy
        if length2 <= 0:
            continue
        u = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/length2))
        distance2 = (x-ax-u*dx)**2+(y-ay-u*dy)**2
        candidate = (distance2, float(stations[i])+u*(float(stations[i+1])-float(stations[i])))
        if candidate[0] < best[0]:
            best = candidate
    return best[1]
