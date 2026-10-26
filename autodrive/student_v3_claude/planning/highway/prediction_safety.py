"""Physical front-body admission for newly time-aligned highway predictions.

Highway forecasts still use the legacy centred hull. A corrected observation
must not release braking while its body already overlaps the actual front of
the ego, whose pose is the rear axle. This current-contact guard supplements
the existing timed forecast and rear admission checks; it does not retune them.
"""
import math

import numpy as np

from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M


# Roundoff at exact SAT contact; this is not a physical clearance margin.
CONTACT_EPSILON_M = 1e-12


def aligned_prediction_front_contact(objects, ego, aligned_indices):
    """Return the first aligned front body overlapping the actual ego now.

    Inputs are the unchanged ordering returned by align_highway_predictions.
    Its verified bodies include the full measured dimensions and prediction
    uncertainty. A rear vehicle remains subject to the existing rear policy.
    """
    angle = math.radians(float(ego['heading_deg']))
    tangent = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-tangent[1], tangent[0]])
    origin = np.array([float(ego['x']), float(ego['y'])])
    centre = origin+EGO_CENTRE_M*tangent
    for index in aligned_indices:
        obj = objects[index]
        position = np.asarray(obj[:2], dtype=float)
        if float((position-origin) @ tangent) <= 0.:
            continue
        yaw = math.radians(float(obj[8]))
        other_tangent = np.array([math.cos(yaw), math.sin(yaw)])
        other_normal = np.array([-other_tangent[1], other_tangent[0]])
        delta = position-centre
        length, width = float(obj[6]), float(obj[7])
        separation = []
        for axis in (tangent, normal, other_tangent, other_normal):
            radius = (EGO_HALF_LENGTH_M*abs(tangent @ axis) +
                      EGO_HALF_WIDTH_M*abs(normal @ axis) +
                      length/2.*abs(other_tangent @ axis) +
                      width/2.*abs(other_normal @ axis))
            separation.append(abs(delta @ axis)-radius)
        if max(separation) <= CONTACT_EPSILON_M:
            return dict(object_id=obj[11], when=0.,
                        reason='aligned front body overlaps actual rear-axle ego',
                        separating_axis_gap_m=float(max(separation)))
    return None
