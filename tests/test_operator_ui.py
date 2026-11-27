from pathlib import Path
import os
import sys
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtWidgets
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
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
        goal, _ = target("WGS84", "22.30001", "114.20001", "", True)
        self.assertEqual((22.30001, 114.20001), (goal.latitude_deg, goal.longitude_deg))
        self.assertTrue(goal.require_geographic_verification)


class FakeGraspGoal(SimpleNamespace):
    PICK = 1
    RELEASE = 2


class OperatorTaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.time = rospy.Time(100)
        clock = patch.object(rospy.Time, "now", side_effect=lambda: self.time)
        clock.start()
        self.addCleanup(clock.stop)
        vision = patch.multiple("operator_ui", GraspAction=object(), GraspGoal=FakeGraspGoal)
        vision.start()
        self.addCleanup(vision.stop)
        with patch.object(Operator, "connect_ros"):
            self.ui = Operator()
        self.ui.timer.stop()
        self.ui.client = Mock()
        self.ui.grasp.client = Mock()
        self.ui.ros_ready = True
        self.ui.nav_server_stamp = self.time
        self.addCleanup(self.ui.close)
        self.status()

    def status(self, **fields):
        self.time += rospy.Duration(.05)
        self.ui.base = UgvStatus(base_online=True, velocity_valid=True,
                                 stop_valid=True, stopped=True, active_velocity_source="zero", **fields)
        self.ui.base.header.stamp = self.ui.base.velocity.header.stamp = self.time
        self.ui.drain_events()

    def select(self):
        self.ui.fields[0].setText("1.25")
        self.ui.fields[1].setText("-0.1")
        self.assertTrue(self.ui.navigation_button.isEnabled())
        self.ui.client.send_goal.assert_not_called()

    def vision_ready(self):
        status = GoalStatusArray()
        status.header.stamp = self.time
        self.ui.events.put(("grasp_status", status))
        self.ui.drain_events()

    def test_navigation_toggles_from_start_to_cancel(self):
        self.select()
        button = self.ui.navigation_button
        self.assertFalse(button.property("running"))
        button.click()
        self.assertTrue(button.property("running"))
        self.ui.client.send_goal.assert_called_once()
        self.assertFalse(self.ui.client.send_goal.call_args.args[0].has_yaw)
        button.click()
        self.assertFalse(button.property("running"))
        self.ui.client.cancel_goal.assert_called_once_with()
        self.assertFalse(button.isEnabled())
        self.ui.client.send_goal.call_args.kwargs["done_cb"](GoalStatus.PREEMPTED, None)
        self.ui.drain_events()
        self.assertTrue(button.isEnabled())

    def test_navigation_button_resets_on_action_terminal(self):
        self.select()
        button = self.ui.navigation_button
        for terminal in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED):
            with self.subTest(terminal=terminal):
                button.click()
                self.assertTrue(button.property("running"))
                self.ui.client.send_goal.call_args.kwargs["done_cb"](terminal, None)
                self.ui.drain_events()
                self.assertFalse(button.property("running"))
                self.assertTrue(button.isEnabled())

    def test_map_nav_goal_starts_directly_without_yaw(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.time
        pose.pose.position.x = 1.5
        pose.pose.orientation.z = 1.0
        self.ui.map_point(pose)
        self.ui.client.send_goal.assert_called_once()
        goal = self.ui.client.send_goal.call_args.args[0]
        self.assertFalse(goal.has_yaw)
        self.assertEqual(goal.local_target.pose.position.x, 1.5)

    def test_map_click_does_not_replace_an_active_ui_goal(self):
        self.select()
        self.ui.navigation_button.click()
        target = self.ui.preview
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.time
        pose.pose.position.x = 9.0
        self.ui.map_point(pose)
        self.ui.client.send_goal.assert_called_once()
        self.assertIs(self.ui.preview, target)

    def test_stale_map_goal_does_not_dispatch(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.time - rospy.Duration(2)
        self.ui.map_point(pose)
        self.ui.client.send_goal.assert_not_called()

    def test_manual_hold_blocks_tasks_without_a_departure_control(self):
        self.select()
        buttons = {widget.text() for widget in self.ui.findChildren(QtWidgets.QPushButton)}
        self.assertTrue({"发车", "停车", "解除驻车"}.isdisjoint(buttons))
        self.status(hold=True)
        self.assertFalse(self.ui.navigation_button.isEnabled())

    def test_stop_work_cancels_navigation_before_stopping_units(self):
        self.select()
        self.ui.navigation_button.click()
        self.ui.services.stop_work = Mock(side_effect=self.ui.client.cancel_goal.assert_called_once_with)
        self.ui.services.stop_requested.emit()
        self.ui.services.stop_work.assert_called_once_with()
        self.assertFalse(self.ui.navigation_button.property("running"))

    def test_pick_and_release_start_cancel_and_reset_independently(self):
        self.vision_ready()
        for button, other, command, terminal in (
            (self.ui.grasp.pick_button, self.ui.grasp.release_button, FakeGraspGoal.PICK, GoalStatus.PREEMPTED),
            (self.ui.grasp.release_button, self.ui.grasp.pick_button, FakeGraspGoal.RELEASE, GoalStatus.ABORTED),
        ):
            with self.subTest(command=command):
                self.ui.grasp.client.reset_mock()
                button.click()
                self.assertTrue(button.property("running"))
                self.assertFalse(other.isEnabled())
                self.assertEqual(self.ui.grasp.client.send_goal.call_args.args[0].command, command)
                if command == FakeGraspGoal.PICK:
                    button.click()
                    self.assertFalse(button.property("running"))
                    self.ui.grasp.client.cancel_goal.assert_called_once_with()
                self.ui.grasp.client.send_goal.call_args.kwargs["done_cb"](terminal, None)
                self.ui.drain_events()
                self.assertFalse(button.property("running"))
                self.assertTrue(other.isEnabled())

    def test_pick_starts_vision_and_cancel_drops_waiting_goal(self):
        self.ui.services.ensure_vision = Mock(return_value=True)
        self.ui.drain_events()
        button = self.ui.grasp.pick_button
        self.assertTrue(button.isEnabled())
        button.click()
        self.assertTrue(button.property("running"))
        self.ui.services.ensure_vision.assert_called_once_with()
        self.ui.grasp.client.send_goal.assert_not_called()
        button.click()
        self.assertFalse(button.property("running"))
        self.vision_ready()
        self.ui.grasp.client.send_goal.assert_not_called()

    def test_release_waits_for_vision_action_before_sending(self):
        self.ui.services.ensure_vision = Mock(return_value=True)
        self.ui.drain_events()
        button = self.ui.grasp.release_button
        self.assertTrue(button.isEnabled())
        button.click()
        self.assertTrue(button.property("running"))
        self.ui.grasp.client.send_goal.assert_not_called()
        goal_link = self.ui.grasp.client.action_client.pub_goal
        goal_link.get_num_connections.return_value = 0
        self.vision_ready()
        self.ui.grasp.client.send_goal.assert_not_called()
        goal_link.get_num_connections.return_value = 1
        self.ui.drain_events()
        self.ui.grasp.client.send_goal.assert_called_once()
        self.assertEqual(self.ui.grasp.client.send_goal.call_args.args[0].command, FakeGraspGoal.RELEASE)

    def test_stopping_navigation_discards_queued_old_action_status(self):
        self.select()
        queued = GoalStatusArray()
        queued.header.stamp = self.time
        self.ui.services.navigation_stopped.emit()
        self.ui.events.put(("navigation_status", queued))
        self.ui.drain_events()
        self.assertFalse(self.ui.navigation_button.isEnabled())

        self.time += rospy.Duration(.1)
        queued.header.stamp = self.time
        self.ui.events.put(("navigation_status", queued))
        self.ui.drain_events()
        self.assertTrue(self.ui.navigation_button.isEnabled())

    def test_waypoint_displays_map_coordinates_without_replacing_target(self):
        self.select()
        selected = self.ui.preview
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = 7.5, -3.25

        self.ui.events.put(("waypoint_point", pose))
        self.ui.drain_events()

        self.assertIn("X 7.500 m，Y -3.250 m", self.ui.waypoint_text.text())
        self.assertIs(self.ui.preview, selected)
        self.ui.client.send_goal.assert_not_called()

    def test_sensor_status_expires_when_messages_stop(self):
        self.ui.events.put(("scan_stamp", self.time))
        self.ui.events.put(("lio_stamp", self.time))
        self.ui.drain_events()
        self.assertIn("就绪", self.ui.scan_text.text())
        self.assertIn("就绪", self.ui.lio_text.text())

        self.time += rospy.Duration(2)
        self.ui.drain_events()
        self.assertIn("等待新数据", self.ui.scan_text.text())
        self.assertIn("等待新数据", self.ui.lio_text.text())

    def test_rtk_readout_keeps_fixed_separate_from_fresh_corrections(self):
        fix = NavSatFix(status=NavSatStatus(status=NavSatStatus.STATUS_GBAS_FIX))
        fix.header.stamp = self.time
        usable = KeyValue(key="fresh_rtk_usable", value="false")
        quality = DiagnosticArray(status=[DiagnosticStatus(name="cup_gnss/receiver", values=[
            usable, KeyValue(key="fix_valid", value="true"), KeyValue(key="fix_type", value="RTK fixed"),
        ])])
        quality.header.stamp = self.time
        for event, msg in (("gnss", quality), ("sigma", quality), ("fix", fix)):
            self.ui.events.put((event, msg))
        self.ui.drain_events()
        self.assertIn("Fixed", self.ui.rtk_headline.text())
        self.assertIn("等待新鲜 RTK", self.ui.rtk_freshness.text())

        usable.value = "true"
        self.ui.drain_events()
        self.assertIn("新鲜 RTK 可用", self.ui.rtk_freshness.text())
