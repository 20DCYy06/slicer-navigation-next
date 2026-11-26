"""Check the generated public action and source/terminal stamp contract."""
import unittest

from geometry_msgs.msg import PoseStamped, TwistStamped
from slicer_interfaces.msg import NavigateToPointFeedback, NavigateToPointGoal
from slicer_interfaces.msg import NavigateToPointResult, UgvStatus
from slicer_interfaces.srv import ControlResponse, ServoResponse


class PublicContract(unittest.TestCase):
    def test_target_modes_and_distinct_results(self):
        self.assertNotEqual(NavigateToPointGoal.LOCAL_POSE, NavigateToPointGoal.WGS84)
        self.assertIsInstance(NavigateToPointGoal().local_target, PoseStamped)
        self.assertEqual(len({
            NavigateToPointResult.CANCELED, NavigateToPointResult.REPLACED,
            NavigateToPointResult.INVALID_GOAL, NavigateToPointResult.STOP_NOT_CONFIRMED,
        }), 4)

    def test_status_and_feedback_publish_validity_separately(self):
        status = UgvStatus()
        feedback = NavigateToPointFeedback()
        self.assertFalse(status.gnss_valid)
        self.assertFalse(status.stop_valid)
        self.assertFalse(status.last_result_valid)
        self.assertFalse(feedback.projected_target_valid)
        self.assertIsInstance(feedback.velocity, TwistStamped)
        self.assertFalse(ControlResponse().accepted)
        self.assertFalse(ServoResponse().acknowledged)


if __name__ == "__main__":
    unittest.main()
