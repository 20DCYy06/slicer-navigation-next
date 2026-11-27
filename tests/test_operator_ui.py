from pathlib import Path
import os
import sys
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtGui, QtWidgets
import rospy
from slicer_interfaces.msg import NavigateToPointResult, UgvStatus

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_operator_ui/scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_operator_ui/src"))
from operator_ui import Operator, grasp_result_text, map_coordinates, result_text, target


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


class OperatorDepartureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.time = rospy.Time(100)
        clock = patch.object(rospy.Time, "now", side_effect=lambda: self.time)
        clock.start()
        self.addCleanup(clock.stop)
        with patch.object(Operator, "connect_ros"):
            self.ui = Operator()
        self.ui.timer.stop()
        self.ui.client = Mock()
        self.ui.ros_ready = True
        self.ui.nav_server_stamp = self.time
        self.ui.request_control = Mock()
        self.addCleanup(self.ui.close)
        self.status(hold=True, epoch=3)

    def status(self, *, hold, epoch, **fields):
        self.time += rospy.Duration(.05)
        self.ui.base = UgvStatus(base_online=True, hold=hold, velocity_valid=True,
                                 stop_valid=True, stopped=True, active_velocity_source="zero",
                                 control_epoch=epoch, **fields)
        self.ui.base.header.stamp = self.ui.base.velocity.header.stamp = self.time
        self.ui.drain_events()

    def select(self):
        self.ui.fields[0].setText("1.25")
        self.ui.fields[1].setText("-0.1")
        self.assertTrue(self.ui.send_button.isEnabled())
        self.ui.request_control.assert_not_called()
        self.ui.client.send_goal.assert_not_called()

    def test_one_tap_waits_for_release_ack_and_fresh_control_status(self):
        self.select()
        self.ui.send_button.click()
        departure = self.ui.departure
        self.ui.request_control.assert_called_once_with("release", departure)
        self.status(hold=False, epoch=4)
        self.ui.client.send_goal.assert_not_called()
        self.ui.events.put(("departure_ack", (departure, True, "")))
        self.ui.drain_events()
        self.ui.client.send_goal.assert_called_once()
        goal = self.ui.client.send_goal.call_args[0][0]
        self.assertEqual(goal.local_target.pose.position.x, 1.25)
        self.ui.drain_events()
        self.ui.client.send_goal.assert_called_once()

    def test_stop_while_releasing_discards_goal_even_after_late_ack(self):
        self.select()
        self.ui.send_button.click()
        departure = self.ui.departure
        self.ui.hold_button.click()
        self.ui.events.put(("departure_ack", (departure, True, "")))
        self.status(hold=False, epoch=4)
        self.ui.client.send_goal.assert_not_called()
        self.ui.request_control.assert_called_with("hold")

    def test_external_authority_change_blocks_departure(self):
        self.select()
        self.ui.send_button.click()
        departure = self.ui.departure
        self.ui.events.put(("departure_ack", (departure, True, "")))
        self.status(hold=False, epoch=5)
        self.ui.client.send_goal.assert_not_called()
        self.ui.request_control.assert_called_with("hold")

    def test_closing_while_releasing_cancels_departure_and_parks(self):
        self.select()
        self.ui.send_button.click()
        departure = self.ui.departure
        close = QtGui.QCloseEvent()
        self.ui.closeEvent(close)
        self.assertFalse(close.isAccepted())
        self.assertTrue(departure.canceled.is_set())
        self.ui.request_control.assert_called_with("hold")
        self.ui.events.put(("departure_ack", (departure, True, "")))
        self.status(hold=False, epoch=4)
        self.ui.client.send_goal.assert_not_called()

    def test_closing_after_release_ack_still_waits_for_parking(self):
        self.select()
        self.ui.send_button.click()
        departure = self.ui.departure
        self.ui.events.put(("departure_ack", (departure, True, "")))
        self.ui.drain_events()
        close = QtGui.QCloseEvent()
        self.ui.closeEvent(close)
        self.assertFalse(close.isAccepted())
        self.assertTrue(departure.canceled.is_set())
        self.ui.request_control.assert_called_with("hold")
        self.ui.client.send_goal.assert_not_called()
