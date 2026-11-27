#!/usr/bin/env python3
"""Cancel live public actions and check post-Hold brake feedback before a service stop."""

from collections import deque
import sys
import time

from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import rospy
from slicer_interfaces.msg import UgvStatus
from slicer_interfaces.srv import Control


ACTIVE = (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING)


def wait_until(predicate, deadline, reason):
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise RuntimeError(reason)


def fresh(stamp, now):
    return not stamp.is_zero() and 0 <= (now - stamp).to_sec() <= .5


class Action:
    def __init__(self, name):
        self.statuses = deque(maxlen=1)
        self.subscriber = rospy.Subscriber(f"{name}/status", GoalStatusArray, self.statuses.append, queue_size=1)
        self.cancel = rospy.Publisher(f"{name}/cancel", GoalID, queue_size=5)

    def active(self):
        if self.statuses:
            return {value.goal_id.id for value in self.statuses[-1].status_list
                    if value.status in ACTIVE and value.goal_id.id}
        return set()

    def connected(self):
        return bool(self.cancel.get_num_connections())

    def cancel_goals(self, goals):
        for goal_id in goals:
            self.cancel.publish(GoalID(id=goal_id))


def quiesce():
    deadline = time.monotonic() + 8
    states = deque(maxlen=1)
    status_subscriber = rospy.Subscriber("/ugv/status", UgvStatus, states.append, queue_size=1)
    navigation = Action("/ugv/navigate")
    vision = Action("/ugv/grasp")
    topics = {name for name, _type in rospy.get_published_topics()}

    def updated():
        return (states and fresh(states[-1].header.stamp, rospy.Time.now())
                and ("/ugv/navigate/status" not in topics or navigation.statuses)
                and ("/ugv/grasp/status" not in topics or vision.statuses))

    wait_until(updated, deadline, "fresh vehicle or action status unavailable")
    nav_goals = navigation.active() | {states[-1].goal_id, states[-1].nav_goal_id} - {""}
    vision_goals = vision.active()
    wait_until(lambda: ((not nav_goals or navigation.connected()) and
                        (not vision_goals or vision.connected())),
               deadline, "an active action has no reachable cancel endpoint")
    navigation.cancel_goals(nav_goals)
    vision.cancel_goals(vision_goals)
    rospy.wait_for_service("/ugv/control/hold", timeout=max(.1, deadline - time.monotonic()))
    result = rospy.ServiceProxy("/ugv/control/hold", Control)()
    if not result.accepted:
        raise RuntimeError("Hold was refused: " + result.reason)
    held_at = rospy.Time.now()

    def confirmed():
        navigation.cancel_goals(navigation.active())
        vision.cancel_goals(vision.active())
        status = states[-1]
        now = rospy.Time.now()
        return (fresh(status.header.stamp, now) and status.hold and status.stop_valid and status.stopped
                and status.velocity_valid and fresh(status.velocity.header.stamp, now)
                and status.velocity.header.stamp > held_at and status.active_velocity_source == "zero"
                and not (status.goal_id or status.nav_goal_id or navigation.active() or vision.active()))

    wait_until(confirmed, deadline, "no fresh measured stop with Hold and inactive actions")
    status_subscriber.unregister()
    print("Hold and fresh measured stop confirmed.")


def main():
    rospy.init_node("slicer_service_quiesce", anonymous=True)
    try:
        quiesce()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as error:
        print(f"Service change refused: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
