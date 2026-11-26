"""ROS adapter for the geographic session owned by the navigation process."""

import math
from threading import RLock

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from slicer_interfaces.msg import UgvStatus

from .alignment import AlignmentLimits, PlanarPose
from .geographic_session import GeographicSession, GeographicUnavailable
from .geographic_transform import GeoPoint
from .gnss_adapter import Fix, FixAdapter, FixQuality


def _time(stamp_ns):
    return rospy.Time(*divmod(stamp_ns, 1_000_000_000))


def _pose(message):
    value = message.pose.pose
    q = value.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.02:
        raise ValueError("odometry quaternion is not unit length")
    sine = 2.0 * (q.w * q.z + q.x * q.y) / norm ** 2
    cosine = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2) / norm ** 2
    return PlanarPose(message.header.stamp.to_nsec(), value.position.x, value.position.y,
                      math.atan2(sine, cosine))


class RosGeographicSession:
    def __init__(self):
        self._lock = RLock()
        self.operational = bool(rospy.get_param("~enable_geographic_navigation", False))
        self._fusion_enabled = self.operational and bool(rospy.get_param("~enable_gnss_fusion", False))
        config = rospy.get_param("~geographic", {})
        origin = config.get("origin")
        self._lio_frame = config.get("lio_frame", "odom")
        self._lio_child_frame = config.get("lio_child_frame", "base_link")
        self._map_frame = "map"
        fix_age_ns = int(config.get("max_fix_age_s", 2.5) * 1e9)
        self._fix_adapter = FixAdapter(frame=config.get("antenna_frame", "gnss_link"),
                                       max_age_ns=fix_age_ns,
                                       assumed_std_m=config.get("fallback_position_std_m", 0.5))
        self._session = GeographicSession(
            config.get("antenna_offset_xy_m", (0.065, 0.0)), GeoPoint(**origin) if origin else None,
            max_fix_age_ns=fix_age_ns, max_pose_gap_ns=int(config.get("max_pose_gap_s", 0.5) * 1e9),
            max_innovation_m=config.get("max_innovation_m", 3.0),
            max_site_radius_m=config.get("max_site_radius_m", 1000.0),
            limits=AlignmentLimits(**config.get("alignment", {})),
        )
        self._last_tick_ns = 0
        self._status_pub = rospy.Publisher("/ugv/internal/geographic_status", UgvStatus, queue_size=1)
        self._xy_pub = rospy.Publisher("/ugv/localization/gnss_xy", PoseWithCovarianceStamped, queue_size=10)
        self._subscribers = [
            rospy.Subscriber(config.get("fix_topic", "/gnss/fix"), NavSatFix, self._fix, queue_size=40),
            rospy.Subscriber(config.get("quality_topic", "/gnss/fix_quality"), DiagnosticArray,
                             self._quality, queue_size=40),
            rospy.Subscriber(config.get("map_pose_topic", "/odometry/global"), Odometry,
                             self._map_pose, queue_size=50),
            rospy.Subscriber(config.get("lio_pose_topic", "/lio/odom"), Odometry,
                             self._lio_pose, queue_size=50),
        ]
        self._timer = rospy.Timer(rospy.Duration(0.1), self._tick, reset=True)

    def _fix(self, message):
        received = rospy.Time.now().to_nsec()
        try:
            altitude = message.altitude if math.isfinite(message.altitude) else None
            point = GeoPoint(message.latitude, message.longitude, altitude)
        except ValueError:
            point = None
        with self._lock:
            self._fix_adapter.add_fix(Fix(message.header.stamp.to_nsec(), received, point,
                                          message.status.status, message.header.frame_id,
                                          tuple(message.position_covariance), message.position_covariance_type))

    def _quality(self, message):
        received = rospy.Time.now().to_nsec()
        for status in message.status:
            if status.name != "cup_gnss/receiver":
                continue
            quality = FixQuality(message.header.stamp.to_nsec(), received,
                                 {pair.key: pair.value for pair in status.values})
            with self._lock:
                self._fix_adapter.add_quality(quality)
            return

    def _map_pose(self, message):
        self._append_pose(message, self._map_frame, "base_link", self._session.add_map_pose)

    def _lio_pose(self, message):
        self._append_pose(message, self._lio_frame, self._lio_child_frame, self._session.add_lio_pose)

    def _append_pose(self, message, frame, child, append):
        if (message.header.frame_id, message.child_frame_id) != (frame, child):
            return
        try:
            pose = _pose(message)
        except ValueError:
            return
        with self._lock:
            append(pose)

    def _healthy(self, now_ns):
        latest = self._session.latest_map_observation
        return (self._session.version != 0 and self._session.can_prepare(now_ns)
                and (latest is None or latest.innovation_m <= self._session.max_innovation_m))

    @property
    def healthy(self):
        with self._lock:
            return self._healthy(rospy.Time.now().to_nsec())

    @property
    def ready(self):
        return self.operational and self.healthy

    @property
    def can_prepare(self):
        with self._lock:
            return self.operational and self._session.can_prepare(rospy.Time.now().to_nsec())

    @property
    def version(self):
        with self._lock:
            return self._session.version

    def begin_preparation(self):
        with self._lock:
            if self.operational:
                self._session.begin_preparation(rospy.Time.now().to_nsec())

    def candidate_ready(self):
        with self._lock:
            return self.operational and self._session.candidate_ready(rospy.Time.now().to_nsec())

    def commit(self, stopped):
        with self._lock:
            return self.operational and self._session.commit(stopped, rospy.Time.now().to_nsec())

    def cancel_preparation(self):
        with self._lock:
            self._session.cancel_preparation()

    def validate_site(self, latitude, longitude, altitude_ellipsoid_m=None):
        with self._lock:
            self._session.validate_site(latitude, longitude, altitude_ellipsoid_m)

    def project(self, latitude, longitude, altitude_ellipsoid_m=None, yaw_enu_rad=None):
        with self._lock:
            if not self.operational:
                raise GeographicUnavailable("Geographic navigation requires an explicitly enabled replay or field profile")
            return self._session.project(latitude, longitude, altitude_ellipsoid_m, yaw_enu_rad)

    def verify(self, projected, after_stamp_ns, tolerance_m):
        with self._lock:
            return self._session.verify(projected, after_stamp_ns, rospy.Time.now().to_nsec(), tolerance_m)

    def _publish_observations(self, observations):
        for observation in observations:
            message = PoseWithCovarianceStamped()
            message.header.stamp = _time(observation.stamp_ns)
            message.header.frame_id = self._map_frame
            message.pose.pose.position.x, message.pose.pose.position.y = observation.x, observation.y
            message.pose.pose.orientation.w = 1.0
            for index, value in zip((0, 1, 6, 7), observation.covariance):
                message.pose.covariance[index] = value
            for index in (14, 21, 28, 35):
                message.pose.covariance[index] = 1e6
            self._xy_pub.publish(message)

    def _status(self, now):
        now_ns = now.to_nsec()
        session = self._session
        message = UgvStatus()
        message.header.stamp = now
        message.transform_version = session.version
        fix = session.latest_fix
        if fix:
            message.gnss_fix_type = fix.fix_type
            message.gnss_valid = fix.valid and fix.fresh(now_ns, session.max_fix_age_ns)
            message.gnss_stamp = _time(fix.fix.stamp_ns)
            message.gnss_received_stamp = _time(fix.fix.received_ns)
        message.alignment_state = message.ALIGNMENT_UNAVAILABLE
        message.geographic_reason = session.reason
        if not self.operational:
            message.geographic_reason = "Geographic navigation is disabled pending localization replay or validation"
        elif session.version:
            message.alignment_state = message.ALIGNMENT_DEGRADED
            if not session.fix_fresh(now_ns):
                message.geographic_reason = "No fresh RTK Fixed position"
            elif not self._healthy(now_ns):
                message.geographic_reason = "Synchronized positioning unavailable or GNSS innovation too large"
            elif not fix.covariance_measured:
                message.geographic_reason = "RTK using conservative assumed horizontal uncertainty"
            else:
                message.alignment_state = message.ALIGNMENT_READY
                message.geographic_reason = ""
        return message

    def _tick(self, _event):
        now = rospy.Time.now()
        stamp_ns = now.to_nsec()
        with self._lock:
            if stamp_ns < self._last_tick_ns:
                self._fix_adapter.reset()
                self._session.invalidate("ROS clock moved backwards")
            self._last_tick_ns = stamp_ns
            observations = []
            for fix in self._fix_adapter.poll(stamp_ns):
                observations.extend(self._session.observe(fix, stamp_ns))
                if not fix.usable:
                    observations.clear()
            observations.extend(self._session.process_pending(stamp_ns))
            if self._fusion_enabled and self._healthy(stamp_ns):
                self._publish_observations(observations)
            self._status_pub.publish(self._status(now))
