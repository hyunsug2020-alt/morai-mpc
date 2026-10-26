"""JSON for status topics and decision logs: a numpy value never stops the planner.

follow-20261010T055542Z: the highway speed plan's recovery_brake_retained was a
numpy bool (an array comparison); the status topic's json.dumps raised TypeError
inside the publish call and the planner thread exited at 68 km/h.
"""
import numpy as np


def json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)
