"""One immutable geographic alignment for a fixed navigation map session."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional, Tuple

from .alignment import AlignmentLimits, AlignmentPair, fit_trajectory, interpolate_pose
from .geographic_transform import EnuFrame, GeoPoint, finite, rotate_xy, wrap_yaw


class GeographicUnavailable(ValueError):
    pass


@dataclass(frozen=True)
class ProjectedGoal:
    original: GeoPoint
    yaw_enu_rad: Optional[float]
    x: float
    y: float
    yaw: Optional[float]
    transform_version: int


@dataclass(frozen=True)
class ArrivalCheck:
    valid: bool = False
    arrived: bool = False
    error_m: Optional[float] = None
    stamp_ns: int = 0
    reason: str = ""


@dataclass(frozen=True)
class MapObservation:
    stamp_ns: int
    x: float
    y: float
    covariance: Tuple[float, float, float, float]
    covariance_measured: bool
    innovation_m: float
    base_enu: Tuple[float, float, float]


class GeographicSession:
    def __init__(self, antenna_offset, origin=None, *, max_fix_age_ns=2_500_000_000,
                 max_pose_gap_ns=500_000_000, max_innovation_m=3.0,
                 max_site_radius_m=1000.0, preparation_timeout_ns=60_000_000_000,
                 limits=AlignmentLimits()):
        if len(antenna_offset) != 2 or not all(finite(value) for value in antenna_offset):
            raise ValueError("horizontal antenna installation must contain finite x and y")
        if not all(finite(value) and value > 0 for value in
                   (max_fix_age_ns, max_pose_gap_ns, max_innovation_m, max_site_radius_m)):
            raise ValueError("invalid geographic session bounds")
        if type(preparation_timeout_ns) is not int or preparation_timeout_ns <= 0:
            raise ValueError("preparation timeout must be positive nanoseconds")
        self.antenna_offset = tuple(antenna_offset)
        self.frame = EnuFrame(origin) if origin else None
        self.max_fix_age_ns = max_fix_age_ns
        self.max_pose_gap_ns = max_pose_gap_ns
        self.max_innovation_m = max_innovation_m
        self.max_site_radius_m = max_site_radius_m
        self.preparation_timeout_ns = preparation_timeout_ns
        self.limits = limits
        self.map_poses = deque(maxlen=600)
        self.lio_poses = deque(maxlen=600)
        self.pairs = deque()
        self._pending = deque(maxlen=256)
        self._fit = None
        self.transform = None
        self._version_count = 0
        self._committed_stamp = 0
        self._last_fit_stamp = 0
        self._last_paired_stamp = 0
        self._map_from_lio_yaw = None
        self._receiver_session = ""
        self.preparing = False
        self._preparation_start_ns = 0
        self.latest_fix = None
        self.latest_map_observation = None
        self.latest_measured_map_observation = None
        self.reason = "Geographic alignment required"

    @property
    def version(self):
        return self._version_count if self.transform is not None else 0

    def invalidate(self, reason):
        self.cancel_preparation()
        self.transform = None
        self._map_from_lio_yaw = None
        self.latest_fix = None
        self.latest_map_observation = None
        self.latest_measured_map_observation = None
        self._pending.clear()
        self._last_paired_stamp = 0
        self.map_poses.clear()
        self.lio_poses.clear()
        self.reason = reason

    def _add_pose(self, buffer, pose, source):
        if buffer:
            previous = buffer[-1]
            age = (pose.stamp_ns - previous.stamp_ns) / 1e9
            distance = math.dist(pose.xy, previous.xy)
            yaw_step = abs(wrap_yaw(pose.yaw - previous.yaw))
            if age == 0 and distance <= 0.01 and yaw_step <= 0.01:
                return
            if (age <= 0 or distance > max(0.75, 2.0 * age)
                    or yaw_step > max(0.7, 3.0 * age)):
                self.invalidate(f"{source} pose reference reset")
        buffer.append(pose)

    def add_map_pose(self, pose):
        self._add_pose(self.map_poses, pose, "Map")

    def add_lio_pose(self, pose):
        self._add_pose(self.lio_poses, pose, "LIO")

    def begin_preparation(self, now_ns=0):
        if self.transform is None and not self.preparing:
            self.preparing = True
            self._preparation_start_ns = now_ns
            self.pairs.clear()
            self._fit = None
            self._last_fit_stamp = 0
            self.reason = "Waiting for paired map and RTK movement"

    def cancel_preparation(self):
        self.preparing = False
        self._preparation_start_ns = 0
        self.pairs.clear()
        self._fit = None
        self._last_fit_stamp = 0
        if self.transform is None:
            self.reason = "Geographic alignment required"

    def fix_fresh(self, now_ns):
        return bool(self.latest_fix and self.latest_fix.usable
                    and self.latest_fix.fresh(now_ns, self.max_fix_age_ns))

    def _pose_pair(self, stamp_ns):
        return (interpolate_pose(self.map_poses, stamp_ns, self.max_pose_gap_ns),
                interpolate_pose(self.lio_poses, stamp_ns, self.max_pose_gap_ns))

    def can_prepare(self, now_ns):
        if not self.fix_fresh(now_ns) or not self.map_poses or not self.lio_poses:
            return False
        for poses in (self.map_poses, self.lio_poses):
            if not 0 <= now_ns - poses[-1].stamp_ns <= self.max_pose_gap_ns:
                return False
        return 0 < self._last_paired_stamp <= now_ns and (
            now_ns - self._last_paired_stamp <= self.limits.max_gap_ns)

    def observe(self, observation, now_ns):
        stamp = observation.fix.stamp_ns
        if self.latest_fix and stamp <= self.latest_fix.fix.stamp_ns:
            return []
        if observation.receiver_session and observation.receiver_session != self._receiver_session:
            self.pairs.clear()
            self._fit = None
            self._last_fit_stamp = 0
            self._pending.clear()
            self.latest_map_observation = None
            self.latest_measured_map_observation = None
            self._last_paired_stamp = 0
            self._receiver_session = observation.receiver_session
        self.latest_fix = observation
        if not observation.usable or not observation.fresh(now_ns, self.max_fix_age_ns):
            self.latest_map_observation = None
            self.latest_measured_map_observation = None
            self._pending.clear()
            self._last_paired_stamp = 0
            self.reason = observation.reason or "GNSS fix is stale"
            return []
        if self.frame is None:
            self.frame = EnuFrame(observation.fix.point)
        self._pending.append(observation)
        return self.process_pending(now_ns)

    def process_pending(self, now_ns):
        result = []
        while self._pending:
            observation = self._pending[0]
            stamp = observation.fix.stamp_ns
            if not observation.fresh(now_ns, self.max_fix_age_ns):
                self._pending.popleft()
                continue
            if (not self.map_poses or not self.lio_poses
                    or min(self.map_poses[-1].stamp_ns, self.lio_poses[-1].stamp_ns) < stamp):
                break
            self._pending.popleft()
            try:
                map_pose, lio_pose = self._pose_pair(stamp)
            except ValueError:
                self.latest_measured_map_observation = None
                self.reason = "GNSS timestamp lacks bracketing map and LIO poses"
                continue
            east, north, up = self.frame.project(observation.fix.point)
            if not all(finite(value) for value in (east, north, up)) or math.hypot(east, north, up) > self.max_site_radius_m:
                self.latest_map_observation = None
                self.latest_measured_map_observation = None
                self._last_paired_stamp = 0
                self.reason = "GNSS observation is outside the geographic site"
                continue
            self._last_paired_stamp = stamp
            antenna_en = east, north
            if self.preparing and stamp >= self._preparation_start_ns:
                # The available displacement depends on elapsed motion, not on
                # how many fixes the receiver produces while the rover moves.
                cutoff = stamp - self.preparation_timeout_ns
                while self.pairs and self.pairs[0].map_pose.stamp_ns < cutoff:
                    self.pairs.popleft()
                self.pairs.append(AlignmentPair(map_pose, antenna_en))
            if self.transform is None or stamp <= self._committed_stamp:
                continue
            corrected = self._map_observation(observation, (east, north, up), map_pose, lio_pose)
            self.latest_map_observation = corrected
            if corrected.covariance_measured:
                self.latest_measured_map_observation = corrected
            if corrected.innovation_m > self.max_innovation_m:
                self.reason = "GNSS innovation exceeds the allowed map correction"
            else:
                result.append(corrected)
        return result

    def candidate_ready(self, now_ns):
        if not self.preparing or not self.can_prepare(now_ns) or not self.pairs:
            return False
        latest = self.pairs[-1].map_pose.stamp_ns
        if latest != self._last_paired_stamp or now_ns - latest > self.limits.max_gap_ns:
            return False
        if self._last_fit_stamp != latest:
            self._last_fit_stamp = latest
            try:
                self._fit = fit_trajectory(self.pairs, self.antenna_offset, self.limits)
                self.reason = "Alignment candidate needs confirmed stop"
            except ValueError as error:
                self._fit = None
                self.reason = str(error)
        return self._fit is not None

    def commit(self, stopped, now_ns):
        if stopped is not True or not self.candidate_ready(now_ns):
            return False
        stamp = self.pairs[-1].map_pose.stamp_ns
        map_pose, lio_pose = self._pose_pair(stamp)
        self._map_from_lio_yaw = wrap_yaw(map_pose.yaw - lio_pose.yaw)
        self.transform = self._fit
        self._version_count += 1
        self._committed_stamp = stamp
        self.cancel_preparation()
        self.reason = "Geographic alignment committed"
        return True

    def validate_site(self, latitude, longitude, altitude_ellipsoid_m=None):
        original = GeoPoint(latitude, longitude, altitude_ellipsoid_m)
        if self.frame is not None:
            enu = self.frame.project(original)
            if not all(finite(value) for value in enu) or math.hypot(*enu) > self.max_site_radius_m:
                raise ValueError("WGS84 target falls outside the geographic site")
        return original

    def project(self, latitude, longitude, altitude_ellipsoid_m=None, yaw_enu_rad=None):
        original = self.validate_site(latitude, longitude, altitude_ellipsoid_m)
        if yaw_enu_rad is not None and not finite(yaw_enu_rad):
            raise ValueError("WGS84 yaw must be finite")
        if self.transform is None:
            raise GeographicUnavailable("Geographic transform unavailable")
        east, north, _ = self.frame.project(original)
        x, y = self.transform.project_xy((east, north))
        yaw = None if yaw_enu_rad is None else wrap_yaw(yaw_enu_rad + self.transform.yaw)
        return ProjectedGoal(original, yaw_enu_rad, x, y, yaw, self.version)

    def _map_observation(self, observation, antenna_enu, map_pose, lio_pose):
        # The precommit yaw offset stays frozen. The incoming yaw is LIO's
        # fixed-session observation, independent from the global EKF update.
        yaw = wrap_yaw(lio_pose.yaw + self._map_from_lio_yaw)
        east, north, up = antenna_enu
        x, y = self.transform.base_xy((east, north), yaw, self.antenna_offset)
        a, b, _, d = observation.covariance_en
        cosine, sine = math.cos(self.transform.yaw), math.sin(self.transform.yaw)
        xx = cosine ** 2 * a - 2 * cosine * sine * b + sine ** 2 * d
        yy = sine ** 2 * a + 2 * cosine * sine * b + cosine ** 2 * d
        xy = cosine * sine * (a - d) + (cosine ** 2 - sine ** 2) * b
        lever_x, lever_y = rotate_xy(self.antenna_offset, yaw)
        lever_east, lever_north = rotate_xy((lever_x, lever_y), -self.transform.yaw)
        yaw_variance = self.transform.heading_std_rad ** 2
        fit_variance = self.transform.rmse_m ** 2
        cross = xy - lever_x * lever_y * yaw_variance
        covariance = (xx + lever_y ** 2 * yaw_variance + fit_variance, cross, cross,
                      yy + lever_x ** 2 * yaw_variance + fit_variance)
        return MapObservation(observation.fix.stamp_ns, x, y, covariance,
                              observation.covariance_measured, math.hypot(x - map_pose.x, y - map_pose.y),
                              (east - lever_east, north - lever_north, up))

    def base_geographic_position(self, now_ns):
        sample = self.latest_map_observation
        if (self.transform is None or sample is None or not self.can_prepare(now_ns)
                or not 0 <= now_ns - sample.stamp_ns <= self.max_fix_age_ns
                or sample.innovation_m > self.max_innovation_m):
            return None
        return self.frame.horizontal_geodetic(sample.base_enu), sample.stamp_ns

    def heading_enu(self, now_ns):
        if self.transform is None or not self.lio_poses:
            return None
        pose = self.lio_poses[-1]
        if not 0 <= now_ns - pose.stamp_ns <= self.max_pose_gap_ns:
            return None
        return wrap_yaw(pose.yaw + self._map_from_lio_yaw - self.transform.yaw), pose.stamp_ns

    def verify(self, goal, after_stamp_ns, now_ns, tolerance_m):
        if not finite(tolerance_m) or tolerance_m <= 0:
            raise ValueError("geographic arrival tolerance must be positive")
        if self.transform is None or goal.transform_version != self.version:
            return ArrivalCheck(reason="Geographic transform changed or became unavailable")
        latest = self.latest_map_observation
        if not self.fix_fresh(now_ns) or latest is None:
            return ArrivalCheck(reason="No fresh, aligned RTK position for geographic verification")
        if latest.stamp_ns <= after_stamp_ns or not 0 <= now_ns - latest.stamp_ns <= self.max_fix_age_ns:
            return ArrivalCheck(reason="Waiting for a new synchronized RTK fix after stopping")
        # Receiver uncertainty may arrive at a lower rate than position.
        measured = self.latest_measured_map_observation
        if (measured is None or measured.stamp_ns <= after_stamp_ns
                or not 0 <= now_ns - measured.stamp_ns <= self.max_fix_age_ns):
            return ArrivalCheck(reason="Receiver uncertainty unavailable for geographic verification")
        xx, xy, _, yy = measured.covariance
        largest_variance = 0.5 * (xx + yy + math.hypot(xx - yy, 2.0 * xy))
        if not largest_variance <= tolerance_m ** 2:
            return ArrivalCheck(reason="Horizontal uncertainty exceeds geographic arrival tolerance")
        error = math.hypot(measured.x - goal.x, measured.y - goal.y)
        return ArrivalCheck(True, error <= tolerance_m, error, measured.stamp_ns,
                            "" if error <= tolerance_m else "Geographic position outside arrival tolerance")
