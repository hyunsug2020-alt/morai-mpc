"""Separate a traffic wait command from a checked prospective launch ceiling.

This metadata does not authorize motion. The live publisher must independently
check current traffic, raw points, people and body geometry before replacing a
traffic-only wait, and must bind the accepted result again to its current plan.
"""
import math
import time


def bind_entry_profile(plan, ceiling_mps, line_id, line_station_m):
    """Attach same-plan evidence without changing its speed or emergency flag."""
    try:
        ceiling, station = float(ceiling_mps), float(line_station_m)
        if (not math.isfinite(ceiling) or ceiling <= 0. or not math.isfinite(station)
                or not isinstance(line_id, str) or not line_id
                or not plan.checks_match_path()
                or any(plan.validation.get(key) is not True
                       for key in ('valid', 'pose_valid', 'steering_valid'))
                or plan.validation.get('blocked') is not False):
            return plan
        metadata = dict(ceiling_mps=ceiling, traffic_wait_only=True,
                        line_id=line_id, line_station_m=station,
                        path_digest=plan.path_digest(), frame_digest=plan.frame_digest())
        return plan.decided(stop_reason=plan.stop_reason, junction_entry_profile=metadata)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return plan


def entry_profile_ceiling(plan, now=None, junction=None):
    """Return a bound prospective ceiling; a zero current command stays zero."""
    try:
        metadata = plan.validation['junction_entry_profile']
        now = time.monotonic() if now is None else float(now)
        ceiling, station = float(metadata['ceiling_mps']), float(metadata['line_station_m'])
        if (metadata.get('traffic_wait_only') is not True
                or not math.isfinite(now) or not math.isfinite(ceiling) or ceiling <= 0.
                or not math.isfinite(station)
                or not isinstance(metadata['line_id'], str) or not metadata['line_id']
                or metadata['path_digest'] != plan.path_digest()
                or metadata['frame_digest'] != plan.frame_digest()
                or not plan.fresh(now, max_age_s=.35) or not plan.checks_match_path()
                or any(plan.validation.get(key) is not True
                       for key in ('valid', 'pose_valid', 'steering_valid'))
                or plan.validation.get('blocked') is not False):
            return None
        if junction is not None:
            if (junction.get('yield_only') is not True or junction.get('following_only')
                    or junction.get('recovery_only') or junction.get('ready') is not True
                    or junction.get('line_id') != metadata['line_id']
                    or not math.isfinite(float(junction['line_station_m']))
                    or abs(float(junction['line_station_m'])-station) > 1e-6):
                return None
        return ceiling
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return None
