"""Private move_base execution. This adapter is called only by the Action worker."""
from queue import SimpleQueue
from threading import Thread

import rospy
from actionlib import ActionClient, CommState
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.msg import Config, DoubleParameter
from dynamic_reconfigure.srv import Reconfigure
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import UInt64
from std_srvs.srv import Empty


MOVE_BASE = "/ugv/internal/move_base"


class ServiceCall:
    def __init__(self):
        self.done = False
        self.value = None
        self.error = None


class SerialServiceCalls:
    """Keep ROS RPCs ordered without ever blocking Action cancellation on a socket."""

    def __init__(self):
        self.pending = SimpleQueue()
        Thread(target=self._run, name="navigation-services", daemon=True).start()

    def submit(self, function, *args):
        call = ServiceCall()
        self.pending.put((call, function, args))
        return call

    def _run(self):
        while not rospy.is_shutdown():
            call, function, args = self.pending.get()
            try:
                call.value = function(*args)
            except Exception as exc:
                call.error = exc
            finally:
                call.done = True


class MoveBaseAdapter:
    def __init__(self):
        self.client = ActionClient(MOVE_BASE, MoveBaseAction)
        self.max_linear = rospy.get_param("/ugv/motion/max_linear_mps")
        self.max_angular = rospy.get_param("/ugv/motion/max_angular_rps")
        self.active = None
        self.retiring = None
        self.sent_at = None
        self.tolerances = rospy.ServiceProxy(MOVE_BASE + "/DWAPlannerROS/set_parameters", Reconfigure)
        self.clear_costmaps = rospy.ServiceProxy(MOVE_BASE + "/clear_costmaps", Empty)
        self.cache_epoch = rospy.Publisher(MOVE_BASE + "/map_cache_epoch", UInt64, queue_size=1)

    def ready(self):
        if self.retiring:
            if self.retiring.get_comm_state() != CommState.DONE:
                return False
            self.retiring = None
        status = self.client.last_status_msg
        if status is None or status.header.stamp.is_zero():
            return False
        age = (rospy.Time.now() - status.header.stamp).to_sec()
        return 0 <= age <= 2.0 and bool(self.client.pub_goal.get_num_connections())

    def configure(self, xy, yaw, preparation=False):
        linear = min(self.max_linear, 0.35) if preparation else self.max_linear
        angular = min(self.max_angular, 0.50) if preparation else self.max_angular
        parameters = Config(doubles=[
            DoubleParameter("xy_goal_tolerance", xy),
            DoubleParameter("yaw_goal_tolerance", yaw),
            DoubleParameter("max_vel_x", linear),
            DoubleParameter("max_vel_trans", linear),
            DoubleParameter("max_vel_theta", angular),
        ])
        response = self.tolerances(parameters)
        applied = {entry.name: entry.value for entry in response.config.doubles}
        if any(applied.get(parameter.name) != parameter.value for parameter in parameters.doubles):
            raise ValueError("DWA rejected the goal tolerance or speed cap")

    def refresh_costmaps(self, version):
        self.clear_costmaps()
        self.cache_epoch.publish(UInt64(version))
        return rospy.Time.now()

    def start(self, target):
        self.sent_at = rospy.Time.now()
        self.active = self.client.send_goal(MoveBaseGoal(target_pose=target))

    def state(self):
        if self.active is None:
            return None, None
        if self.active.get_comm_state() == CommState.DONE:
            return self.active.get_terminal_state(), self.active.get_goal_status_text()
        if self.active.get_goal_status() == GoalStatus.ACTIVE:
            return GoalStatus.ACTIVE, None
        return None, None

    def cancel(self):
        if self.active is not None:
            if self.active.get_comm_state() != CommState.DONE:
                self.active.cancel()
                self.retiring = self.active
            self.active = None
        self.sent_at = None
