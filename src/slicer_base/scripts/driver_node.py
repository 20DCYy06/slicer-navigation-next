#!/usr/bin/env python3
"""The only process that opens or writes the C30D serial connection."""

from concurrent.futures import TimeoutError
from math import cos, isfinite, sin
from pathlib import Path
from threading import Lock
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rospkg
import rospy
import yaml

from slicer_interfaces.msg import UgvStatus
from slicer_interfaces.srv import Servo, ServoResponse
from slicer_base.driver_core import ServoMailbox, StopObservation, WheelSample


BODY_OUTPUT = "/ugv/internal/body_velocity"
DRIVER_STATUS = "/ugv/internal/driver_status"
WHEEL_ODOM = "/ugv/internal/wheel_odom"


class ServoFrames:
    """Observe only servo replies as the pinned chassis parser decodes the link."""
    def __init__(self, parser, mailbox, protocol):
        self.parser = parser
        self.mailbox = mailbox
        self.protocol = protocol

    def feed(self, raw):
        frames = self.parser.feed(raw)
        for kind, _seq, payload in frames:
            if kind == self.protocol.CAPABILITY_REPLY:
                try:
                    pin, _neutral, _watchdog = self.protocol.decode_capability(payload)
                except RuntimeError as error:
                    rospy.logwarn("C30D servo capability rejected: %s", error)
                else:
                    self.mailbox.capability({14: 1, 15: 2}[pin])
            elif kind == 0x87:  # V1.9 echoes the uint16 pulse, with its own frame sequence.
                self.mailbox.acknowledge(payload)
        return frames


def load_hardware():
    override = rospy.get_param("~hardware_root", "")
    installed = Path(rospkg.RosPack().get_path("slicer_base")) / "c30d"
    if override and not Path(override).is_absolute():
        raise ValueError("hardware_root must be an absolute standalone hardware checkout")
    host = Path(override).resolve() / "Host" if override else installed
    for module in (host / "rtk/c30d_link.py", host / "c30d_servo.py"):
        if not module.is_file():
            raise RuntimeError("Pinned C30D dependency is missing at %s" % module)
    sys.path[:0] = [str(host / "rtk"), str(host)]
    import c30d_link
    import c30d_servo
    import chassis_core
    import chassis_recovery

    rospy.loginfo("C30D host modules: %s", host)
    return c30d_link, c30d_servo, chassis_core, chassis_recovery


class BaseDriver:
    def __init__(self, config, motion, mock):
        self.config = config
        self.mock = mock
        self.mailbox = ServoMailbox(ack_timeout=.25)
        self.lock = Lock()
        self.target = (0., 0., float("-inf"))
        self.armed_since = float("inf")
        self.accepting_velocity = False
        self.car = None
        self.recovery = None
        self.latest_velocity = TwistStamped()
        self.velocity_received = float("-inf")
        self.last_body_sample = 0.
        self.feedback_ticks = None
        self.wheel = WheelSample(config["encoder_signs"])
        self.still = StopObservation(motion["stop_linear_mps"], motion["stop_angular_rps"],
                                     motion["stop_window_s"], config["body_feedback_timeout_s"])
        self.status_pub = rospy.Publisher(DRIVER_STATUS, UgvStatus, queue_size=1)
        self.odom_pub = rospy.Publisher(WHEEL_ODOM, Odometry, queue_size=2)
        self.feedback_pub = rospy.Publisher("/ugv/internal/mcu_feedback", DiagnosticArray, queue_size=2)
        self.sub = rospy.Subscriber(BODY_OUTPUT, TwistStamped, self.command, queue_size=1, tcp_nodelay=True)
        self.servo = rospy.Service("/ugv/actuators/servo", Servo, self.servo_request)

    def command(self, message):
        connection = getattr(message, "_connection_header", {}) or {}
        if connection.get("callerid") != "/slicer_velocity_mux" or connection.get("latching", "0") != "0":
            return
        values = message.twist.linear.x, message.twist.angular.z
        now_ros = rospy.Time.now()
        age = (now_ros - message.header.stamp).to_sec()
        timeout = self.config["command_timeout_s"]
        received = time.monotonic()
        with self.lock:
            if received < self.armed_since:
                return
            if message.header.frame_id != "base_link" or message.header.stamp.is_zero():
                return
            if not (0 <= age < timeout and all(isfinite(value) for value in values)):
                self.target = (0., 0., float("-inf"))
                return
            if values == (0., 0.):
                self.accepting_velocity = True
            if self.accepting_velocity:
                self.target = (*values, received + timeout - age)

    def servo_request(self, request):
        if self.mock:
            if request.channel == 1 and 500 <= request.pulse_us <= 2500:
                return ServoResponse(True, False, "simulated forwarding without an MCU reply")
            return ServoResponse(False, False, "channel or pulse outside simulated servo capability")
        future = self.mailbox.submit(request.channel, request.pulse_us, time.monotonic())
        try:
            return ServoResponse(*future.result(timeout=.75))
        except TimeoutError:
            canceled = future.cancel()
            return ServoResponse(False, False, "servo expired before transmission" if canceled
                                 else "driver could not confirm transmission")

    def publish_status(self, online, reason):
        now = time.monotonic()
        status = UgvStatus()
        status.header.stamp = rospy.Time.now()
        status.base_online = online
        status.reason = reason
        status.velocity = self.latest_velocity
        status.velocity_valid = 0 <= now - self.velocity_received <= self.config["body_feedback_timeout_s"]
        status.stop_valid, status.stopped = self.still.state(now)
        self.status_pub.publish(status)

    def update_velocity(self, now, stamp, linear, angular, wheel_mm_s):
        if stamp.is_zero():
            return
        self.latest_velocity = TwistStamped()
        self.latest_velocity.header.stamp = stamp
        self.latest_velocity.header.frame_id = "base_link"
        self.latest_velocity.twist.linear.x = linear
        self.latest_velocity.twist.angular.z = angular
        self.velocity_received = now
        self.still.update(now, stamp.to_sec(), linear, angular, tuple(speed * .001 for speed in wheel_mm_s))

    def publish_odom(self, stamp, pose_twist):
        x, y, yaw, linear, angular = pose_twist
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = sin(yaw / 2)
        message.pose.pose.orientation.w = cos(yaw / 2)
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        for index in (0, 7, 35):
            message.pose.covariance[index] = message.twist.covariance[index] = .1
        for index in (14, 21, 28):
            message.pose.covariance[index] = message.twist.covariance[index] = 1e6
        self.odom_pub.publish(message)

    def publish_mcu_feedback(self, stamp, wheel, body, wheel_recent, body_recent):
        ticks = wheel["tick"], body.get("mcu_us")
        if ticks == self.feedback_ticks:
            return
        self.feedback_ticks = ticks
        diagnostic = DiagnosticStatus(
            name="C30D telemetry", hardware_id=str(self.config["serial_number"]),
            level=DiagnosticStatus.OK if wheel_recent and body_recent else DiagnosticStatus.WARN,
            message="wheel/body samples received",
        )
        values = {
            "wheel_mcu_tick_us": wheel["tick"],
            "wheel_interval_us": wheel["interval_us"],
            "wheel_rx_monotonic_s": self.car.last_rx,
            "wheel_fresh": wheel_recent,
            "wheel_flags": wheel["flags"],
            "wheel_command_ack_seq": wheel["ack"],
            "wheel_raw_counts_abcd": list(wheel["raw"]),
            "wheel_speed_mm_s_abcd": list(wheel["mm_s"]),
            "wheel_pwm_ticks_abcd": list(wheel["pwm"]),
            "wheel_voltage_v": wheel["voltage"],
            "body_mcu_tick_us": body.get("mcu_us"),
            "body_rx_monotonic_s": self.car.last_body_rx,
            "body_fresh": body_recent,
            "body_status_flags": body.get("status_flags"),
            "body_imu_flags": body.get("imu_flags"),
            "body_imu_age_ms": body.get("imu_age_ms"),
            "body_source": body.get("source"),
            "body_vx_mps": body.get("vx_m_s"),
            "body_wz_rps": body.get("wz_rad_s"),
        }
        diagnostic.values = [KeyValue(key=name, value=str(value)) for name, value in values.items()]
        message = DiagnosticArray(status=[diagnostic])
        message.header.stamp = stamp  # ROS host receipt; MCU ticks retain their own clock above.
        self.feedback_pub.publish(message)

    def output(self, now, online):
        with self.lock:
            if not online:
                self.armed_since = float("inf")
                self.accepting_velocity = False
                self.target = (0., 0., float("-inf"))
            elif self.armed_since == float("inf"):
                # The Mux must send a new zero across the readiness boundary.
                # An earlier nonzero stream cannot resume after a driver fault.
                self.armed_since = now
            linear, angular, expires = self.target
            return (linear, angular) if now < expires else (0., 0.)

    def step_hardware(self):
        car = self.car
        car.poll(0)
        now = time.monotonic()
        stamp = rospy.Time.now()
        age = now - car.last_rx
        status = car.status
        flags = status["flags"]
        body = car.body_telemetry or {}
        wheel_recent = 0 <= age <= self.config["wheel_feedback_timeout_s"]
        body_recent = car.body_feedback_fresh(self.config["body_feedback_timeout_s"],
                                              self.config["body_imu_timeout_ms"])
        if body_recent and car.last_body_rx > self.last_body_sample:
            self.last_body_sample = car.last_body_rx
            self.update_velocity(car.last_body_rx, stamp, body["vx_m_s"], body["wz_rad_s"], body["wheel_mm_s"])
        if wheel_recent:
            sample = self.wheel.update((status["tick"], status["raw"]))
            if sample is not None and not stamp.is_zero():
                self.publish_odom(stamp, self.odom.update(*sample))
        else:
            self.wheel.previous = None

        recovery_hold = False
        recovering_telemetry = False
        if not wheel_recent:
            recovering_telemetry = self.recovery.wait_for_telemetry(now, body.get("status_flags"), age)
        else:
            recovery_tools = self.recovery_tools
            body_flags = body.get("status_flags") if body_recent and car.last_body_rx > self.recovery.need_body_after else None
            body_flags = recovery_tools.paired_body_status(body_flags, status["tick"], body.get("mcu_us"))
            recovery_hold, _recovered = self.recovery.update(
                car, now, flags, body_flags, last_body_status=body.get("status_flags"),
                stationary=recovery_tools.stopped(status["mm_s"], status["pwm"]), ack=status["ack"])

        if not wheel_recent:
            reason = "wheel telemetry stale"
        elif not body_recent:
            reason = "body/IMU telemetry stale"
        elif flags & 4 or recovery_hold:
            reason = "MCU fault or recovery hold"
        elif not flags & 8 or flags & (64 | 128):
            reason = "MCU body configuration unavailable"
        elif flags & (16 | 32) != (16 | 32):
            reason = "motor disabled or drive power unavailable"
        else:
            reason = ""
        online = not reason
        # Service processing follows the scheduled body command. No reply wait
        # runs on this loop. Clearing readiness also invalidates the host cache.
        car.body_speed(*self.output(now, online))
        if now - car.last_rx > self.config["telemetry_timeout_s"] and not recovering_telemetry:
            raise RuntimeError("MCU wheel telemetry timeout")
        if now >= self.next_servo_query and self.mailbox.channel is None:
            car.send(self.protocol.CAPABILITY_QUERY, b"")
            self.next_servo_query = now + 2.
        self.mailbox.tick(now, lambda payload: car.send(self.protocol.PULSE_COMMAND, payload))
        self.publish_status(online, reason or "C30D body control ready")
        try:
            self.publish_mcu_feedback(stamp, status, body, wheel_recent, body_recent)
        except rospy.ROSException as error:
            rospy.logwarn_throttle(2, "MCU feedback publication failed: %s", error)

    def run(self):
        try:
            if not self.mock:
                link, self.protocol, chassis, self.recovery_tools = load_hardware()
                self.odom = chassis.Odometry(dict(self.config, port_order=list("ABCD")))
                self.recovery = self.recovery_tools.LinkRecovery(arm_watchdog=True)
                self.car = link.Chassis(self.config)
                self.car.configure(self.config["encoder_signs"])
                self.car.require_body_control()
                ceiling = self.car.set_output_limit_percent(self.config["motor_output_limit_percent"],
                                                           self.config["mcu_pwm_accept_max_permille"])
                rospy.loginfo("Verified MCU output ceiling: %s", ceiling)
                self.car.parser = ServoFrames(self.car.parser, self.mailbox, self.protocol)
                self.next_servo_query = time.monotonic()
            while not rospy.is_shutdown():
                started = time.monotonic()
                if self.mock:
                    stamp = rospy.Time.now()
                    linear, angular = self.output(started, True)
                    self.update_velocity(started, stamp, linear, angular, (linear * 1000,) * 4)
                    self.publish_odom(stamp, (0., 0., 0., linear, angular))
                    self.publish_status(True, "simulated base feedback")
                else:
                    self.step_hardware()
                time.sleep(max(0., 1 / self.config["publish_hz"] - (time.monotonic() - started)))
        finally:
            self.mailbox.close()
            if self.recovery is not None:
                self.recovery.stop_requests()
            try:
                if self.car is not None:
                    self.car.close()
            finally:
                self.still.invalidate()
                self.velocity_received = float("-inf")
                self.publish_status(False, "base driver stopped")


def main():
    rospy.init_node("slicer_base_driver")
    with open(rospy.get_param("~hardware_config"), encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config.get("control_mode") != "body_speed":
        raise ValueError("Only the verified C30D body_speed configuration is supported")
    BaseDriver(config, rospy.get_param("/ugv/motion"), rospy.get_param("~mock")).run()


if __name__ == "__main__":
    main()
