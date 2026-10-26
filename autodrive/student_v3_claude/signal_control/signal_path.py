"""Traffic-light speed cap at the selected route's signal stop line."""
import math

from .signal_logic import requires_stop
from .signal_geometry import FRONT_BUMPER_M, projected_route_station
from .permissions import IGNORE, LEGACY


class SignalPathGate:
    def __init__(self, run_id, max_age=1.0, deceleration=1.5, stop_margin=1.0, reaction_seconds=1.0):
        self.run_id = run_id
        self.max_age = max_age
        self.deceleration = deceleration
        self.stop_margin = stop_margin
        self.reaction_seconds = reaction_seconds
        self.hold_line = None
        self.braking_line = None
        self.braking_target = None
        self.last_cap = None
        self.previous_command_line = None
        self.previous_command_cap = None
        self.permitted_crossing = None
        self.cleared_line = None

    @staticmethod
    def _key(line):
        return line["link_id"], float(line["station"])

    def _crossed_with_permission(self, line, station, now):
        """Retire a passed line before a stale pose can select it again.

        The 20 Hz publisher can pass the line before an older inference pose
        arrives. Reselecting that line clears its camera report and formerly
        latched UNKNOWN/HOLD inside the junction. Only a recent, live permit
        obtained within 5 m before crossing can clear this signal's stop.
        """
        if self.cleared_line is not None:
            gap = self.cleared_line["station"]-station-FRONT_BUMPER_M
            if abs(gap) > 10.:
                self.cleared_line = None  # Retire clearance after leaving the junction or starting a new approach.
        if self.permitted_crossing is not None:
            permitted, frame_wall = self.permitted_crossing
            gap = permitted["station"]-station-FRONT_BUMPER_M
            if not 0. <= now-frame_wall <= self.max_age:
                self.permitted_crossing = None
            elif -5. <= gap < 0.:
                self.cleared_line, self.permitted_crossing = permitted, None
                key = self._key(permitted)
                if self.hold_line is not None and self._key(self.hold_line) == key:
                    self.hold_line = None
                if self.braking_target is not None and self._key(self.braking_target) == key:
                    self.braking_target = None
                if self.braking_line == key:
                    self.braking_line, self.last_cap = None, None
                if self.previous_command_line == key:
                    self.previous_command_line, self.previous_command_cap = None, None
        if self.cleared_line is not None and (line is None or self._key(line) == self._key(self.cleared_line)):
            return self.cleared_line
        return None

    def evaluate(self, report, line, station, now, speed_mps=None):
        result = {"active": False, "state": "UNKNOWN", "raw_state": "UNKNOWN", "fresh": False,
                  "stop_required": False, "cap_mps": None, "reason": None, "run_id": self.run_id}
        cleared = self._crossed_with_permission(line, float(station), float(now))
        if cleared is not None:
            result.update(reason="permitted signal stop line already crossed", phase="CLEARED",
                          cleared_after_permission=True,
                          stop_line_node_id=cleared.get("stop_line_node_id"),
                          traffic_light_id=cleared.get("traffic_light_id"),
                          signal_index=cleared.get("signal_index"), maneuver=cleared.get("maneuver"),
                          permission_policy=cleared.get("permission_policy"),
                          distance_m=cleared["station"]-float(station)-FRONT_BUMPER_M)
            return result
        if self.hold_line is not None:
            held_distance = float(self.hold_line["station"])-float(station)-FRONT_BUMPER_M
            if abs(held_distance) <= 5.:
                line = self.hold_line
            else:
                self.hold_line, self.braking_line, self.last_cap = None, None, None
                self.braking_target = None
                self.previous_command_line, self.previous_command_cap = None, None
        # A small overshoot or localization jump must not make a red stop
        # disappear merely because the route selector now excludes the line.
        if self.hold_line is None and self.braking_target is not None:
            remembered_gap = float(self.braking_target["station"])-float(station)-FRONT_BUMPER_M
            if abs(remembered_gap) <= 5.:
                line = self.braking_target
            elif line is None:
                self.braking_target, self.braking_line, self.last_cap = None, None, None
        if line is None:
            result["reason"] = "no selected signal stop line within 60 m ahead"
            return result
        distance = float(line["station"]) - float(station) - FRONT_BUMPER_M
        if distance < -.3 and self.hold_line is None and self.braking_target is None:
            return result  # Do not stop inside a junction for a signal behind the bumper.
        line_key = (line["link_id"], float(line["station"]))
        margin = float(line.get("stop_margin_m", self.stop_margin))
        maneuver = line.get("maneuver", "straight")
        policy = line.get("permission_policy", LEGACY)
        if policy == IGNORE:
            self.hold_line = self.braking_target = self.braking_line = self.last_cap = None
            self.previous_command_line = self.previous_command_cap = None
            result.update(reason="right-turn signal ignored", permission_policy=IGNORE,
                          signal_index=line.get("signal_index"),
                          stop_line_node_id=line.get("stop_line_node_id"),
                          traffic_light_id=line.get("traffic_light_id"), maneuver=maneuver,
                          distance_m=distance)
            return result
        report = report if isinstance(report, dict) else {}
        try:
            fresh = (report.get("fresh") is True and report.get("source") == "live" and
                     report.get("run_id") == self.run_id and bool(self.run_id) and
                     0 <= now-float(report.get("frame_wall", 0)) <= self.max_age and
                     0 <= now-float(report.get("time", 0)) <= self.max_age)
        except (ValueError, TypeError):
            fresh = False
        state = report.get("state", "UNKNOWN") if fresh else "UNKNOWN"
        raw = report.get("raw_state", "UNKNOWN") if fresh else "UNKNOWN"
        stop = (requires_stop(raw, maneuver, policy) or requires_stop(state, maneuver, policy)
                or report.get('permission_stable') is not True)
        if not stop and 0. <= distance <= 5.:
            self.permitted_crossing = (dict(line), float(report["frame_wall"]))
        elif (stop and distance >= 0. and raw != "UNKNOWN" and self.permitted_crossing is not None
              and self._key(self.permitted_crossing[0]) == line_key):
            # An observed prohibiting color before entry cancels the earlier permit.
            self.permitted_crossing = None
        # UNKNOWN must stop even if green was seen in a previous frame.
        if not stop:
            self.hold_line, self.braking_line, self.last_cap = None, None, None
            self.braking_target = None
            self.previous_command_line, self.previous_command_cap = None, None
            if distance < -.3:
                return result
        # Measured MORAI speed follows the published cap with delay. Budget that
        # travel before braking: v*t + v*v/(2*a) <= distance - stop_margin.
        # The 1.5 m/s² approach allows about 10 km/h more at 50-60 m than
        # the former 0.8 m/s² envelope. Keep a full second of reaction and
        # one metre at the bumper for the delayed final speed command.
        usable = max(0., distance-margin)
        delay_term = self.deceleration*self.reaction_seconds
        # Confirmed permission returns control to the driving planner immediately.
        # A green signal must not impose a stop-line braking/creep envelope.
        cap = (math.sqrt(delay_term*delay_term + 2*self.deceleration*usable)-delay_term) if stop else None
        # A red/unknown approach has one decreasing command envelope. Localization
        # jitter must never increase the target or launch a second creep phase.
        if stop:
            if self.previous_command_line == line_key and self.previous_command_cap is not None:
                cap = min(cap, self.previous_command_cap)
            if self.braking_line == line_key and self.last_cap is not None:
                cap = min(cap, self.last_cap)
            self.braking_line, self.last_cap = line_key, cap
            self.braking_target = dict(line)
        phase = "BRAKE" if stop else "ALLOW"
        if stop and (self.hold_line is not None or distance <= margin or cap <= .05):
            cap, phase = 0., "HOLD"
            self.hold_line = dict(line)
            self.last_cap = 0.
        if stop:
            self.previous_command_line, self.previous_command_cap = line_key, cap
        reason = "traffic light %s: %s at stop line %.1f m (%s)" % (
            raw, "STOP" if stop else "ALLOW", distance, maneuver)
        result.update(active=True, state=state, raw_state=raw, fresh=fresh,
                      stop_required=stop, cap_mps=cap, reason=reason,
                      line_id=line["link_id"], line_station=line["station"], distance_m=distance,
                      stop_line_source=line.get("stop_line_source", "map"),
                      stop_line_node_id=line.get("stop_line_node_id"),
                      stop_line_xy=line.get("stop_line_xy"), front_bumper_m=FRONT_BUMPER_M,
                      label=line.get("label"), traffic_light_id=line.get("traffic_light_id"),
                      left_permitted=bool(not stop and maneuver == "left"),
                      signal_permitted=not stop,
                      stop_margin_m=margin, deceleration_mps2=self.deceleration,
                      reaction_seconds=self.reaction_seconds, phase=phase,
                      clearing_after_green=False,
                      control_owner="traffic_light" if stop else "driving_model",
                      maneuver=maneuver, permission_policy=policy, signal_index=line.get("signal_index"),
                      run_id=self.run_id)
        return result

    @staticmethod
    def apply(plan, current_cap_mps, emergency=False):
        """Combine caps without releasing a stop requested by another planner."""
        cap = plan.get("cap_mps")
        if cap is None:
            return current_cap_mps, emergency
        return min(current_cap_mps, cap), emergency or cap <= .05
