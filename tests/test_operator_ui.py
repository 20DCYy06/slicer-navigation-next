from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from slicer_interfaces.msg import NavigateToPointResult

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_operator_ui/scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_operator_ui/src"))
from operator_ui import grasp_result_text, map_coordinates, result_text, target


class OperatorResultTest(unittest.TestCase):
    def test_grasp_cancellation_preserves_recovery_message(self):
        result = SimpleNamespace(success=False, needs_release=True, message="抓取任务已取消")
        text = grasp_result_text(GoalStatus.PREEMPTED, result)
        self.assertIn("已取消", text)
        self.assertIn("需要释放或恢复", text)

    def test_missing_grasp_result_keeps_recovery_state_unknown(self):
        self.assertIn("终态未知", grasp_result_text(GoalStatus.LOST, None))

    def test_lost_action_without_result_does_not_claim_stop(self):
        text = result_text(GoalStatus.LOST, None)
        self.assertIn("未收到导航结果", text)
        self.assertIn("停稳未确认", text)

    def test_geographic_result_shows_valid_error_and_measured_stop(self):
        result = NavigateToPointResult(code=NavigateToPointResult.SUCCESS, stop_valid=True, stopped=True,
                                       position_error_valid=True, position_error_m=.13365, error_source="gnss_xy")
        text = result_text(GoalStatus.SUCCEEDED, result)
        self.assertIn("已停稳", text)
        self.assertIn("0.13 m (gnss_xy)", text)

    def test_external_takeover_can_report_measured_motion(self):
        result = NavigateToPointResult(code=NavigateToPointResult.HELD, stop_valid=True, stopped=False)
        self.assertIn("仍在运动", result_text(GoalStatus.ABORTED, result))

    def test_map_click_accepts_only_finite_map_coordinates(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = 1.2, -2.1
        self.assertEqual((1.2, -2.1), map_coordinates(pose))
        pose.header.frame_id = "odom"
        with self.assertRaises(ValueError):
            map_coordinates(pose)
        pose.header.frame_id = "map"
        pose.pose.position.x = float("nan")
        with self.assertRaises(ValueError):
            map_coordinates(pose)

    def test_wgs84_edit_uses_decimal_degrees_and_requests_verification(self):
        goal, _ = target("WGS84", "22.30001", "114.20001", "", "", True)
        self.assertEqual((22.30001, 114.20001), (goal.latitude_deg, goal.longitude_deg))
        self.assertTrue(goal.require_geographic_verification)
