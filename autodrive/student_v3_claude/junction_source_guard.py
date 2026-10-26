"""Bind a provisional junction launch to the latest complete detector frame."""
from contextlib import contextmanager
import hashlib
import json
import math
import time


def frame_identity(status, ready=True):
    """Ignore changing receipt age, but include same-stamp body revisions."""
    try:
        stamp = float(status['stamp'])
        if (not ready or not math.isfinite(stamp) or stamp <= 0.
                or status.get('ready') is not True
                or status.get('active') is not True
                or not isinstance(status.get('vehicles'), list)):
            return None
        value = {key: status[key] for key in
                 ('vehicles', 'ghost_ids', 'unknown_ids', 'inherited_ids') if key in status}
        encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
        return dict(stamp=stamp, digest=hashlib.sha256(encoded.encode()).hexdigest())
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def current_frame(receiver, expected):
    """A changed complete frame requests one complete recheck, never a TTL renewal."""
    try:
        _, ready, status = receiver.snapshot()
        latest = frame_identity(status, ready)
        if latest is None or expected is None:
            return False, 'invalid'
        if latest['stamp'] < expected['stamp']:
            return False, 'backwards'
        return (True, 'same') if latest == expected else (False, 'changed')
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False, 'invalid'


def receipt_evidence(receiver, expected):
    """Report actual receiver ownership only under its own callback lock."""
    lock = getattr(receiver, 'lock', None)
    if lock is None:
        return None
    with lock:
        now = time.monotonic()
        try:
            received = float(receiver.received)
            same = frame_identity(receiver.record) == expected
            return dict(checked_at=now, receiver_received_at=received,
                        receipt_age_s=now-received, captured_frame_is_current=same)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None


@contextmanager
def publication_guard(provider, plan):
    """Lock complete-frame comparison, entry consumption and both ROS writes.

    The publisher already owns its command lock. The detector callback takes
    only receiver.lock, so the lock order is command -> receiver -> controller.
    Already committed following commands carry no launch request and retain
    their existing body/person/raw checks without a new frame-change stop.
    """
    request = plan.validation.get('junction_source_launch') if plan is not None else None
    if request is None:
        yield True
        return
    receiver = getattr(provider.runtime, 'vehicle_receiver', None)
    lock = getattr(receiver, 'lock', None)
    if lock is None:
        provider.reject_source_publication(plan, 'junction source has no atomic receiver barrier')
        yield False
        return
    with lock:
        now = time.monotonic()
        try:
            age = now-float(receiver.received)
            status = receiver.record
            identity = frame_identity(status, ready=0. <= age <= .35)
            plan.validation['junction_source_publication'] = dict(
                checked_at=now, receiver_received_at=float(receiver.received),
                receipt_age_s=age, expected=request.get('frame'), actual=identity)
            valid = (identity is not None and identity == request['frame']
                     and math.isfinite(float(request['expires_at']))
                     and plan.checked_at <= now <= float(request['expires_at'])
                     and plan.allowed_mps > 0. and not plan.emergency
                     and plan.fresh(now) and plan.checks_match_path()
                     and all(plan.validation.get(key) is True for key in
                             ('valid', 'pose_valid', 'steering_valid'))
                     and plan.validation.get('blocked') is False)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            provider.reject_source_publication(plan, 'junction source changed before publication')
            yield False
            return
        if request.get('kind') != 'launch':
            yield True
            return
        controller = getattr(provider.runtime, 'stop_line_controller', None)
        parent = getattr(provider.runtime, 'gap_planner', None)
        state_lock = getattr(controller, '_state_lock', None)
        if (state_lock is None or not callable(getattr(parent, 'latch_junction_commit', None))
                or not callable(getattr(provider.traffic, 'latch_junction_commit', None))):
            provider.reject_source_publication(plan, 'junction launch controller barrier unavailable')
            yield False
            return
        with state_lock:
            previous_approach = bool(controller.yield_approach)
            try:
                # This current whole-path GO owns the reservation, including
                # when the older GUI frame still described a pure traffic WAIT.
                controller.yield_approach = True
                consumed = controller.consume_yield_launch(request['line_id'],
                    request['line_station_m'], request['front_station_m'],
                    request['speed_mps'], request['pose_age_s'], expires_at=request['expires_at'])
            except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                consumed = False
            if not consumed:
                controller.yield_approach = previous_approach
                provider.reject_source_publication(plan, 'junction launch no longer authorized')
                yield False
                return
            parent.latch_junction_commit()
            provider.traffic.latch_junction_commit()
            sequence = getattr(provider, '_source_launch_sequence', 0)+1
            provider._source_launch_sequence = provider._source_launch_transaction = sequence
            request.update(consumed=True, published=False, transaction_id=sequence)
            plan.validation['live_safety']['junction_traffic'].update(state='ENTER', gap_checked=True)
            try:
                yield True
            except BaseException:
                provider.reject_source_publication(plan, 'junction publication failed')
                raise
