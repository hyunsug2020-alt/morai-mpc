"""A measured, bounded speed trajectory shared by highway prediction and control.

Trajectory times, ego speed/acceleration and object velocities are simulator
units: the runtime converts measured object tuples (sim_time.to_sim_objects)
and sets ``time_scale`` (the real-time factor) for the ego's own acceleration
estimate, which is timed by wall-clock pose stamps.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

from sim_time import clip_rtf
from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M
from planning.highway.trajectory_geometry import normalize_path, projected_halves


STEP_S = .1
CONTROL_LOOKAHEAD_S = .4
ACCEL_CONTROL_LOOKAHEAD_S = 1.6
# Longer predictions select lanes. Only a near-term footprint conflict changes
# the speed command; distant uncertainty is reconsidered on the next decision.
TRAFFIC_CONTROL_HORIZON_S = 3.
COMFORT_DECEL_MPS2 = 2.5
JERK_MPS3 = 4.
# Following gap target (front bumper to the lead's rear): max(MIN_GAP_M,
# HEADWAY_S * v). The preview compares the lead one STEP_S ahead with the ego's
# current distance, so the settled gap behind an equal-speed lead is one step of
# travel shorter (following_gap_m): 0.65 s, i.e. 7.2 m at 40 km/h, 10.8 m at
# 60 km/h, 16.3 m at 90 km/h (was 0.9 s / 5 m -> 0.8 s settled; user 2026-10-10:
# shorter, moderately). The reaction reserve (FOLLOW_RESPONSE_S + observation
# age + jerk ramp), the closing cap at the comfort deceleration, the footprint
# forecast and the braking escalation in HighwayPlanner.plan_speed are unchanged.
HEADWAY_S = .75
MIN_GAP_M = 4.
GAP_GAIN = .45
FOLLOW_RESPONSE_S = .35
# The live follower uses throttle = .35 * speed error. Its 3 m/s² full-pedal
# model must stay inside the trajectory's acceleration assumption.
FOLLOWER_ACCEL_GAIN = .35
FOLLOWER_FULL_ACCEL_MPS2 = 3.
# Highway acceleration assumption = the full-pedal model, so the reference may
# lead the measured speed by 1/FOLLOWER_ACCEL_GAIN (full throttle). The shared
# 2.5 m/s² launch bound held the pedal at 0.78-0.83: 1.4-1.45 m/s² per wall
# second at 55-110 km/h, against 2.06 at full pedal (follow-20261010T140051Z,
# mpc_emissions). Forecasts, the command preview and the measured-acceleration
# clip use this same bound, so no checked trajectory is slower than the car.
HIGHWAY_ACCEL_MPS2 = FOLLOWER_FULL_ACCEL_MPS2
FOLLOWER_BRAKE_GAIN = .45
FOLLOWER_FULL_BRAKE_MPS2 = 8.
FOLLOWER_SPEED_DEADBAND_MPS = .15
FOLLOWER_REFERENCE_RISE_MPS2 = 3.
FOLLOWER_RESPONSE_S = .25


def _cached_command_preview(model):
    """Reuse scalar actuator forecasts within one immutable speed decision.

    The command model reads the actual reference, initial motion and time grid;
    later nominal speeds/distances do not affect execution. Keep output metadata
    in the key too, and return independent arrays. Braking-limit admission stays
    with the caller so an unrestricted query cannot approve a restricted one.
    """
    cache = {}
    stats = dict(queries=0, computations=0, hits=0)

    def check(trajectory):
        stats['queries'] += 1
        times = trajectory.times
        key = (times.dtype.str, times.shape, times.tobytes(), trajectory.reference_mps,
               float(trajectory.speeds[0]), float(trajectory.accelerations[0]),
               trajectory.ceiling_mps, trajectory.braking_limit_mps2)
        executed = cache.get(key)
        if executed is None:
            stats['computations'] += 1
            executed = model.command_preview(trajectory)
            executed = replace(executed, times=executed.times.copy(),
                               distances=executed.distances.copy(), speeds=executed.speeds.copy(),
                               accelerations=executed.accelerations.copy())
            cache[key] = executed
        else:
            stats['hits'] += 1
        return replace(executed, times=executed.times.copy(),
                       distances=executed.distances.copy(), speeds=executed.speeds.copy(),
                       accelerations=executed.accelerations.copy())

    check.stats = stats
    return check


def following_gap_m(speed_mps):
    """Settled bumper gap behind a lead at the same speed (simulator units)."""
    speed = max(0., float(speed_mps))
    return max(MIN_GAP_M, HEADWAY_S*speed)-STEP_S*speed


def clearance_forecast_bound(clear_m, query_m):
    """Keep measured hits; a completed no-hit query is not a stopping line.

    Infinity means no obstacle was found in this query, not infinite sensor
    coverage. Invalid clearance remains blocked and short paths stay bounded.
    """
    clear_m, query_m = float(clear_m), float(query_m)
    if not math.isfinite(clear_m) or not math.isfinite(query_m) or query_m <= 0.:
        return 0.
    return float("inf") if clear_m >= query_m-1e-6 else max(0., clear_m)


@dataclass
class SpeedTrajectory:
    times: np.ndarray
    distances: np.ndarray
    speeds: np.ndarray
    accelerations: np.ndarray
    goal_mps: float
    ceiling_mps: float = float("inf")
    braking_limit_mps2: float = COMFORT_DECEL_MPS2
    reference_override_s: float = None
    execution_reference_checked: bool = False
    recovery_brake_retained: bool = False
    acceleration_reference_limited: bool = False
    braking_reference_extended: bool = False
    braking_reference_trials: int = 0

    def acceleration_reference_time(self, reference):
        """Keep the P-controller acceleration bound inside this trajectory."""
        ceiling = float(self.speeds[0]) + HIGHWAY_ACCEL_MPS2 / (
            FOLLOWER_ACCEL_GAIN * FOLLOWER_FULL_ACCEL_MPS2)
        reached = np.flatnonzero((self.times <= reference) & (self.speeds >= ceiling))
        if not len(reached):
            # The interpolated endpoint can cross the bound between samples.
            end = min(int(np.searchsorted(self.times, reference)), len(self.times)-1)
            if float(np.interp(reference, self.times, self.speeds)) < ceiling:
                return reference
            at = end
        else:
            at = int(reached[0])
        before = max(0, at-1)
        span = float(self.speeds[at]-self.speeds[before])
        if span <= 0.:
            return float(self.times[before])
        return float(self.times[before] + (ceiling-self.speeds[before]) / span *
                     (self.times[at]-self.times[before]))

    @property
    def reference_time_s(self):
        if self.reference_override_s is not None:
            return min(float(self.times[-1]), max(0., self.reference_override_s))
        immediate = float(np.interp(CONTROL_LOOKAHEAD_S, self.times, self.speeds))
        # The follower's throttle feedback is weaker than its brake feedback.
        # A short acceleration reference never reaches the forecast acceleration
        # (MORAI check: clear highway stayed below 60 km/h). Keep the braking
        # lookahead short, but let throttle track the same plan further ahead.
        if immediate > self.speeds[0] + .01:
            reference = min(ACCEL_CONTROL_LOOKAHEAD_S, float(self.times[-1]))
            # A stronger throttle reference must not skip a checked switch to
            # braking for a closing leader, wall or highway exit. Use the peak
            # before that switch, never a later recovery beyond the constraint.
            braking = np.flatnonzero((self.times > CONTROL_LOOKAHEAD_S) &
                (self.times <= reference) & (self.accelerations < -.01))
            if len(braking):
                reference = max(CONTROL_LOOKAHEAD_S, float(self.times[braking[0]-1]))
            return self.acceleration_reference_time(reference)
        if self.goal_mps > self.speeds[0] + .01:
            # A measured brake ramp can outlast the short reference even after
            # the checked road clears. Publishing that lower speed would ask
            # the follower to brake again and renew the measured deceleration.
            # Find recovery in this same jerk-bounded trajectory; never create
            # it when traffic/stopping constraints keep the trajectory slower.
            recovered = np.flatnonzero((self.times <= TRAFFIC_CONTROL_HORIZON_S) &
                (self.speeds > self.speeds[0] + .01) & (self.accelerations > 0.))
            if len(recovered):
                reference = min(TRAFFIC_CONTROL_HORIZON_S, float(self.times[-1]),
                    float(self.times[recovered[0]]) + ACCEL_CONTROL_LOOKAHEAD_S)
                speed = float(np.interp(reference, self.times, self.speeds))
                acceleration = float(np.interp(reference, self.times, self.accelerations))
                if speed > self.speeds[0] + .01 and acceleration > 0.:
                    return self.acceleration_reference_time(reference)
        return CONTROL_LOOKAHEAD_S

    @property
    def reference_mps(self):
        # Measured momentum may temporarily exceed the mission limit; the
        # command must never ask the follower to accelerate past that limit.
        acceleration_ceiling = float(self.speeds[0]) + HIGHWAY_ACCEL_MPS2 / (
            FOLLOWER_ACCEL_GAIN * FOLLOWER_FULL_ACCEL_MPS2)
        return min(self.ceiling_mps, acceleration_ceiling,
                   float(np.interp(self.reference_time_s, self.times, self.speeds)))

    def diagnostics(self):
        return dict(goal_kmh=round(self.goal_mps * 3.6, 2),
                    reference_kmh=round(self.reference_mps * 3.6, 2),
                    reference_time_s=self.reference_time_s,
                    acceleration_mps2=round(float(np.interp(
                        self.reference_time_s, self.times, self.accelerations)), 3),
                    horizon_s=float(self.times[-1]),
                    distance_m=round(float(self.distances[-1]), 2),
                    braking_limit_mps2=self.braking_limit_mps2,
                    execution_reference_checked=self.execution_reference_checked,
                    recovery_brake_retained=self.recovery_brake_retained,
                    acceleration_reference_limited=self.acceleration_reference_limited,
                    braking_reference_extended=self.braking_reference_extended,
                    braking_reference_trials=self.braking_reference_trials,
                    reference_checked=True)

    def prefix(self, horizon_s):
        end = int(np.searchsorted(self.times, horizon_s + 1e-9, side="right"))
        return SpeedTrajectory(self.times[:end], self.distances[:end],
                               self.speeds[:end], self.accelerations[:end],
                               self.goal_mps, self.ceiling_mps, self.braking_limit_mps2,
                               self.reference_override_s, self.execution_reference_checked,
                               self.recovery_brake_retained, self.acceleration_reference_limited,
                               self.braking_reference_extended, self.braking_reference_trials)


class HighwaySpeedPlanner:
    """Predict from actual speed; retain measured acceleration across decisions.

    Front-vehicle headway is evaluated along the same spatial path at every
    trajectory step. A safety decision therefore checks the gradual speed
    change that will be commanded, rather than an unrelated cruise target.
    """
    def __init__(self, comfort_decel_mps2=COMFORT_DECEL_MPS2, jerk_mps3=JERK_MPS3,
                 emergency_jerk_mps3=JERK_MPS3, body_shift_m=EGO_CENTRE_M):
        self.body_shift_m = float(body_shift_m)
        self.front_m = EGO_HALF_LENGTH_M+self.body_shift_m
        self.rear_m = EGO_HALF_LENGTH_M-self.body_shift_m
        self.previous = None
        self.acceleration = 0.
        self.cache = {}
        self.comfort_decel_mps2 = float(comfort_decel_mps2)
        self.jerk_mps3 = float(jerk_mps3)
        self.emergency_jerk_mps3 = float(emergency_jerk_mps3)
        if not all(math.isfinite(v) and v > 0. for v in
                   (self.comfort_decel_mps2, self.jerk_mps3, self.emergency_jerk_mps3)):
            raise ValueError("Highway braking and jerk limits must be finite and positive")
        self.motion_paths = {}
        # uid -> footprint heading (deg) of long lane followers, set per frame
        # by the runtime from HighwayMotionEvidence.lane_headings.
        self.stable_yaws = {}
        # Wall-second observation ages: used only as reaction reserves, where a
        # wall duration is an upper bound of its simulated duration.
        self.sensor_age_s = 0.
        self.fusion_sensor_age_s = 0.
        self.static_sensor_age_s = 0.
        # Simulator real-time factor for wall-stamped ego samples (begin()).
        self.time_scale = 1.
        self.end_distance_m = None
        self.end_speed_mps = 60./3.6
        self.end_buffer_m = 0.

    def object_positions(self, obj, times):
        """Measured motion follows a validated highway centreline when available."""
        key = obj[11] if len(obj)>11 else id(obj)
        native = self.motion_paths.get(key)
        if native is None:
            velocity = obj[9:11] if len(obj)>=11 else (0., 0.)
            return np.asarray(obj[:2]) + times[:, None]*np.asarray(velocity)
        path, stations, start, speed, lateral, lateral_speed = native
        distances = start+speed*times
        points = np.column_stack([np.interp(distances, stations, path[:, k]) for k in (0, 1)])
        index = np.clip(np.searchsorted(stations, distances), 1, len(path)-1)
        tangent = path[index]-path[index-1]
        tangent /= np.maximum(1e-6, np.linalg.norm(tangent, axis=1, keepdims=True))
        extra = np.maximum(0., distances-stations[-1])+np.minimum(0., distances-stations[0])
        points += tangent*extra[:, None]
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        return points+normal*(lateral+lateral_speed*times)[:, None]

    def object_yaws(self, obj, times):
        """Footprint headings: the box yaw, or its lane's for a long lane follower."""
        key = obj[11] if len(obj)>11 else id(obj)
        angle = math.radians(float(self.stable_yaws.get(key, obj[8])))
        native = self.motion_paths.get(key)
        if native is None:
            return np.full(len(times), angle)
        path, stations, start, speed, _, _ = native
        index = np.clip(np.searchsorted(stations, start+speed*times), 1, len(path)-1)
        tangent = path[index]-path[index-1]
        at = int(np.clip(np.searchsorted(stations, start), 1, len(path)-1))
        initial = path[at]-path[at-1]
        return angle+np.arctan2(tangent[:, 1], tangent[:, 0])-math.atan2(initial[1], initial[0])

    def begin(self, ego):
        stamp = float(ego.get("stamp", 0.))
        speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
        if self.previous is not None:
            elapsed = stamp - self.previous[0]
            if .03 <= elapsed <= .5:
                # Pose stamps are wall clock; speed is simulator speed. The
                # acceleration the trajectory continues is per simulated second.
                simulated = elapsed * clip_rtf(self.time_scale)
                measured = np.clip((speed - self.previous[1]) / simulated,
                                   -6., HIGHWAY_ACCEL_MPS2)
                self.acceleration += min(1., elapsed / .2) * (measured - self.acceleration)
            elif elapsed < 0. or elapsed > .5:
                self.acceleration = 0.
        self.previous = stamp, speed
        self.cache.clear()

    def command_preview(self, trajectory):
        """Check the scalar command as well as the desired speed trajectory.

        The C++ command-rise limit, pedal gains and deadband are explicit.
        Full-pedal acceleration/braking and actuator lag are plant estimates;
        this additional check cannot replace fresh measured motion feedback.
        In particular, a positive recovery reference releases current braking
        sooner than the nominal jerk ramp, so its crossing footprint differs.
        """
        reference = trajectory.reference_mps
        speed, acceleration = float(trajectory.speeds[0]), float(trajectory.accelerations[0])
        emitted = speed
        speeds, distances, accelerations = [speed], [0.], [acceleration]
        for elapsed in np.diff(trajectory.times):
            emitted = min(reference, emitted+FOLLOWER_REFERENCE_RISE_MPS2*elapsed)
            error = emitted-speed
            throttle = min(1., max(0., FOLLOWER_ACCEL_GAIN*error)) if (
                error>FOLLOWER_SPEED_DEADBAND_MPS) else 0.
            brake = min(1., max(0., -FOLLOWER_BRAKE_GAIN*error)) if (
                error<-FOLLOWER_SPEED_DEADBAND_MPS) else 0.
            acceleration += min(1., elapsed/FOLLOWER_RESPONSE_S)*(
                FOLLOWER_FULL_ACCEL_MPS2*throttle-FOLLOWER_FULL_BRAKE_MPS2*brake-acceleration)
            next_speed = max(0., speed+acceleration*elapsed)
            distances.append(distances[-1]+(speed+next_speed)*elapsed/2.)
            speeds.append(next_speed)
            accelerations.append(acceleration)
            speed = next_speed
        return SpeedTrajectory(trajectory.times, np.asarray(distances), np.asarray(speeds),
                               np.asarray(accelerations), reference,
                               trajectory.ceiling_mps, trajectory.braking_limit_mps2)

    def leaders(self, path, ego, objects, times):
        original_path = path
        key = id(path), id(objects), len(times), float(times[-1])
        cached = self.cache.get(key)
        if cached is not None and cached[0] is path and cached[1] is objects:
            return cached[2]
        path, arc = normalize_path(path)
        # Two-metre samples keep repeated candidate-speed checks inexpensive.
        stations = np.arange(0., arc[-1] + .01, 2.)
        stations = np.unique(np.r_[stations, arc[-1]])
        samples = np.column_stack([np.interp(stations, arc, path[:, k]) for k in (0, 1)])
        tangent = np.gradient(samples, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        yaw = math.radians(float(ego["heading_deg"]))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        output = []
        for obj in objects:
            if obj[4] != "npc" or (len(obj) > 14 and not obj[14]):
                continue
            velocity = np.asarray(obj[9:11], dtype=float) @ rotation
            if np.linalg.norm(velocity) < .3:
                continue
            position = (np.asarray(obj[:2]) - [ego["x"], ego["y"]]) @ rotation
            future = (self.object_positions(obj, times)-[ego['x'], ego['y']]) @ rotation
            nearest = np.argmin(np.sum((future[:, None] - samples[None]) ** 2, axis=2), axis=1)
            delta = future - samples[nearest]
            lateral = np.abs(np.sum(delta * normal[nearest], axis=1))
            along_speed = tangent[nearest] @ velocity
            aligned = along_speed >= np.linalg.norm(velocity) * math.cos(math.radians(30.))
            relative_yaw = self.object_yaws(obj, times)-yaw-np.arctan2(
                tangent[nearest, 1], tangent[nearest, 0])
            half_length, half_width = projected_halves(float(obj[6]), float(obj[7]), relative_yaw)
            on_path = aligned & (lateral <= EGO_HALF_WIDTH_M + half_width + .3)
            station = stations[nearest] + np.sum(delta * tangent[nearest], axis=1)
            observed_age = float(obj[15]) if len(obj) > 15 else 0.
            source_age = (self.sensor_age_s if obj[3] == 'lidar_car'
                          else self.fusion_sensor_age_s)
            output.append((station, along_speed, on_path, half_length,
                           max(0., source_age, observed_age) if math.isfinite(observed_age)
                           else float('inf')))
        self.cache[key] = original_path, objects, output
        return output

    def preview(self, path, ego, objects, goal_mps, horizon_s=7.,
                static_clear_m=float("inf"), static_mover_mps=0., standoff_m=12.,
                decel_mps2=None, ceiling_mps=float("inf"), speed_profile=None):
        """``speed_profile``: optional (stations, speeds) limit along the path
        (highway_planner.geometry_speed_profile), applied at each forecast
        distance instead of one horizon-wide curvature goal."""
        decel_mps2 = self.comfort_decel_mps2 if decel_mps2 is None else decel_mps2
        times = np.arange(0., horizon_s + .01, STEP_S)
        speed = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
        goal = min(ceiling_mps, max(0., float(goal_mps)))
        # The comfort limit bounds the desired brake, not current momentum.
        # Clamping an active -4.5 m/s² brake to -1.2 m/s² approved a path
        # that could pass a neighbour only if the ego instantly stopped braking.
        # Keep the same measured acceleration bound used by begin().
        acceleration = float(np.clip(self.acceleration, -6., HIGHWAY_ACCEL_MPS2))
        distance = 0.
        distances, speeds, accelerations = [0.], [speed], [acceleration]
        leaders = self.leaders(path, ego, objects, times)
        for i, stamp in enumerate(times[1:], 1):
            target = goal
            if speed_profile is not None:
                target = min(target, float(np.interp(distance, speed_profile[0], speed_profile[1])))
            if self.end_distance_m is not None:
                available = max(0., self.end_distance_m-self.end_buffer_m-distance)
                target = min(target, math.sqrt(self.end_speed_mps**2+
                                              2.*self.comfort_decel_mps2*available))
            for station, lead_speed, on_path, half_length, observed_age in leaders:
                gap = station[i] - distance - self.front_m - half_length[i]
                if on_path[i] and station[i] > distance:
                    gap_error = gap - max(MIN_GAP_M, speed * HEADWAY_S)
                    # Approach at cruise while there is room, then match the
                    # measured leader continuously. The proportional gap term
                    # alone can request an unbrakeable closing speed and force
                    # the footprint veto to alternate cruise and emergency.
                    # Reserve the actual observation delay and the current
                    # acceleration's jerk ramp before spending the surplus gap.
                    delay = FOLLOW_RESPONSE_S + observed_age
                    delay += max(0., acceleration) / self.jerk_mps3
                    available = max(0., gap_error)
                    response = decel_mps2 * delay
                    closing_cap = (2. * decel_mps2 * available /
                        (math.sqrt(response * response + 2. * decel_mps2 * available) + response)
                        if available > 0. and math.isfinite(response) else 0.)
                    target = min(target, max(0., lead_speed[i] + min(
                        GAP_GAIN * gap_error, closing_cap)))
            if math.isfinite(static_clear_m):
                available = static_clear_m + static_mover_mps * stamp - distance
                target = min(target, max(0., math.sqrt(1.6 ** 2 +
                    8. * max(0., available - standoff_m) + static_mover_mps ** 2) - 1.6))
            desired = max(-decel_mps2, min(HIGHWAY_ACCEL_MPS2, (target - speed) / .5))
            # Strong braking retains the existing jerk allowance when a late
            # conflict cannot be handled by the highway comfort trajectory.
            jerk = (self.emergency_jerk_mps3
                    if decel_mps2 > self.comfort_decel_mps2 or acceleration < -self.comfort_decel_mps2
                    else self.jerk_mps3)
            acceleration += max(-jerk * STEP_S,
                                min(jerk * STEP_S, desired - acceleration))
            next_speed = max(0., speed + acceleration * STEP_S)
            distance += (speed + next_speed) * STEP_S / 2.
            distances.append(distance)
            speeds.append(next_speed)
            accelerations.append(acceleration)
            speed = next_speed
        return SpeedTrajectory(times, np.asarray(distances), np.asarray(speeds),
                               np.asarray(accelerations), goal, ceiling_mps, decel_mps2)

    def preview_many(self, path, ego, objects, goals_mps, horizon_s=7.,
                     static_clear_m=float("inf"), static_mover_mps=0., standoff_m=12.,
                     decel_mps2=None, ceiling_mps=float("inf"), speed_profile=None):
        """Evaluate independent pace goals with the same scalar forecast rules.

        Highway rear-gap searches try many goals on one unchanged spatial path.
        Batch only the goals: each time step retains the measured acceleration,
        jerk, following, static stopping, curvature and exit constraints used by
        ``preview``. Returned trajectories own their arrays independently.
        """
        goals = np.asarray([min(ceiling_mps, max(0., float(goal)))
                            for goal in goals_mps], dtype=float)
        if not len(goals):
            return []
        decel = self.comfort_decel_mps2 if decel_mps2 is None else decel_mps2
        times = np.arange(0., horizon_s + .01, STEP_S)
        initial = max(0., float(ego.get("speed_kmh", 0.))) / 3.6
        speed = np.full(len(goals), initial)
        acceleration = np.full(len(goals), float(np.clip(
            self.acceleration, -6., HIGHWAY_ACCEL_MPS2)))
        distance = np.zeros(len(goals))
        distances = np.zeros((len(goals), len(times)))
        speeds = np.full_like(distances, initial)
        accelerations = np.full_like(distances, acceleration[0])
        leaders = self.leaders(path, ego, objects, times)
        for i, stamp in enumerate(times[1:], 1):
            target = goals.copy()
            if speed_profile is not None:
                target = np.minimum(target, np.interp(
                    distance, speed_profile[0], speed_profile[1]))
            if self.end_distance_m is not None:
                available = np.maximum(0., self.end_distance_m-self.end_buffer_m-distance)
                target = np.minimum(target, np.sqrt(self.end_speed_mps**2 +
                    2.*self.comfort_decel_mps2*available))
            for station, lead_speed, on_path, half_length, observed_age in leaders:
                # Match scalar preview's activity gate. Rear/adjacent leaders
                # cannot constrain any pace until they are ahead on this path.
                if not on_path[i]:
                    continue
                active = station[i] > distance
                if not active.any():
                    continue
                gap = station[i]-distance-self.front_m-half_length[i]
                gap_error = gap-np.maximum(MIN_GAP_M, speed*HEADWAY_S)
                delay = FOLLOW_RESPONSE_S+observed_age+np.maximum(0., acceleration)/self.jerk_mps3
                available = np.maximum(0., gap_error)
                response = decel*delay
                closing_cap = np.zeros(len(goals))
                valid = (available > 0.) & np.isfinite(response)
                closing_cap[valid] = (2.*decel*available[valid] /
                    (np.sqrt(response[valid]*response[valid] +
                             2.*decel*available[valid])+response[valid]))
                following = np.maximum(0., lead_speed[i] + np.minimum(
                    GAP_GAIN*gap_error, closing_cap))
                target = np.where(active, np.minimum(target, following), target)
            if math.isfinite(static_clear_m):
                available = static_clear_m+static_mover_mps*stamp-distance
                target = np.minimum(target, np.maximum(0., np.sqrt(1.6**2 +
                    8.*np.maximum(0., available-standoff_m) + static_mover_mps**2)-1.6))
            desired = np.maximum(-decel, np.minimum(HIGHWAY_ACCEL_MPS2,
                                                   (target-speed)/.5))
            jerk = np.where((decel > self.comfort_decel_mps2) |
                            (acceleration < -self.comfort_decel_mps2),
                            self.emergency_jerk_mps3, self.jerk_mps3)
            acceleration += np.maximum(-jerk*STEP_S,
                                        np.minimum(jerk*STEP_S, desired-acceleration))
            next_speed = np.maximum(0., speed+acceleration*STEP_S)
            distance += (speed+next_speed)*STEP_S/2.
            distances[:, i], speeds[:, i], accelerations[:, i] = distance, next_speed, acceleration
            speed = next_speed
        return [SpeedTrajectory(times.copy(), distances[k].copy(), speeds[k].copy(),
                                accelerations[k].copy(), float(goal), ceiling_mps, decel)
                for k, goal in enumerate(goals)]
