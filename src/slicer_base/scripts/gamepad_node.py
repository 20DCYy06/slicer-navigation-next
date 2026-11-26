#!/usr/bin/env python3
"""Route the vehicle's Linux joystick through the base Mux and software e-stop."""

import os
import struct
import time

from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import Bool
import yaml

from slicer_base.gamepad import Gamepad


EVENT = struct.Struct("<IhBB")


def run_device(device, gamepad, takeover_pub, velocity_pub, stop_pub):
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    sending = False
    try:
        while not rospy.is_shutdown():
            if not os.path.exists(device):
                raise OSError("gamepad removed")
            # Limit pending input processing per cycle so a burst cannot
            # delay the stop path or prevent command expiry.
            for _ in range(64):
                try:
                    data = os.read(fd, EVENT.size)
                except BlockingIOError:
                    break
                if len(data) != EVENT.size:
                    raise OSError("gamepad disconnected")
                _, value, kind, axis = EVENT.unpack(data)
                if gamepad.event(kind, axis, value, time.monotonic()):
                    stop_pub.publish(Bool(data=True))
            live, velocity = gamepad.output(time.monotonic())
            if live or sending:
                if live:
                    takeover_pub.publish(Bool(data=True))
                message = Twist()
                message.linear.x, message.angular.z = velocity
                velocity_pub.publish(message)
            sending = live
            time.sleep(.02)
    finally:
        os.close(fd)
        if sending:
            velocity_pub.publish(Twist())


def main():
    rospy.init_node("slicer_gamepad")
    with open(rospy.get_param("~config"), encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    device = rospy.get_param("~device", "") or config["device"]
    takeover_pub = rospy.Publisher("/ugv/internal/manual_override", Bool, queue_size=1)
    velocity_pub = rospy.Publisher("/teleop/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
    stop_pub = rospy.Publisher("/cup_nav/emergency_stop", Bool, queue_size=1, latch=True)
    while not rospy.is_shutdown():
        try:
            run_device(device, Gamepad(config), takeover_pub, velocity_pub, stop_pub)
        except OSError as error:
            rospy.logwarn_throttle(3, "Gamepad unavailable at %s: %s", device, error)
            time.sleep(.25)


if __name__ == "__main__":
    main()
