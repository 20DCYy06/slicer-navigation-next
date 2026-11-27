"""Check the generated public action and source/terminal stamp contract."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
from unittest.mock import patch

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from slicer_interfaces.msg import NavigateToPointFeedback, NavigateToPointGoal
from slicer_interfaces.msg import NavigateToPointResult, UgvStatus
from slicer_interfaces.srv import ControlResponse, ServoResponse

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_navigation/scripts"))
from navigate_server import NavigationServer, Task, map_pose


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


class VerifiedArrivalContract(unittest.TestCase):
    def setUp(self):
        self.server = NavigationServer.__new__(NavigationServer)
        self.server.verified_stop_reserve_m = 0.02

    def task(self, mode=NavigateToPointGoal.WGS84, verify=True, xy=0.0, preparation=False):
        goal = NavigateToPointGoal(mode=mode, require_geographic_verification=verify,
                                   xy_tolerance_m=xy, has_yaw=True, yaw_tolerance_rad=0.24)
        return Task(None, 0.0, 10.0, "arrival", goal, preparation=preparation)

    def test_verified_geographic_goal_reserves_private_xy_only(self):
        task = self.task()
        self.server.last_config = None
        self.server.move_base = Mock()
        self.server.services = Mock()
        self.server.services.submit.return_value.done = False
        self.server.drive(task, 1.0, None)

        self.server.services.submit.assert_called_once()
        configure, xy, yaw, preparation = self.server.services.submit.call_args.args
        self.assertIs(configure, self.server.move_base.configure)
        self.assertAlmostEqual(xy, 0.18)
        self.assertEqual(yaw, 0.24)
        self.assertFalse(preparation)
        self.assertEqual(self.server.tolerances(task), (0.20, 0.24))

    def test_small_requested_tolerance_keeps_a_positive_private_tolerance(self):
        task = self.task(xy=0.006)
        self.assertAlmostEqual(self.server.planner_tolerances(task)[0], 0.003)
        self.assertEqual(self.server.tolerances(task)[0], 0.006)

    def test_other_private_tolerances_match_public_tolerances(self):
        for task in (self.task(mode=NavigateToPointGoal.LOCAL_POSE, verify=False),
                     self.task(verify=False, xy=0.12),
                     self.task(preparation=True, xy=0.12)):
            with self.subTest(mode=task.goal.mode, verify=task.goal.require_geographic_verification,
                              preparation=task.preparation):
                self.assertEqual(self.server.planner_tolerances(task), self.server.tolerances(task))

    def test_stationary_map_gate_keeps_the_public_xy_tolerance(self):
        self.server.stop_confirmed = Mock(return_value=True)
        self.server.finish = Mock()
        pose = Odometry()
        self.server.current_pose = Mock(return_value=pose)

        task = self.task()
        task.stop_code = NavigateToPointResult.SUCCESS
        task.target = map_pose(0.0, 0.0, 0.0)
        pose.pose.pose = map_pose(0.195, 0.0, 0.0).pose
        self.server.confirm_stop(task, 1.0)
        self.assertEqual(task.phase, NavigateToPointFeedback.VERIFYING)
        self.server.finish.assert_not_called()

        task = self.task()
        task.stop_code = NavigateToPointResult.SUCCESS
        task.target = map_pose(0.0, 0.0, 0.0)
        pose.pose.pose = map_pose(0.201732, 0.0, 0.0).pose
        self.server.confirm_stop(task, 1.0)
        self.server.finish.assert_called_once_with(
            task, NavigateToPointResult.NO_PATH, "Final stopped pose is outside the requested tolerance")


class InterruptedActionContract(unittest.TestCase):
    def setUp(self):
        self.server = NavigationServer.__new__(NavigationServer)
        self.server.stop_freshness = .5
        self.server.stop_timeout = 3.
        self.server.current_pose = Mock(return_value=None)
        self.server.release = Mock()
        self.server.ownership = Mock()
        now = patch("navigate_server.rospy.Time.now", return_value=rospy.Time.from_sec(10.))
        now.start()
        self.addCleanup(now.stop)

    def test_an_external_estop_is_specific_even_with_base_disabled(self):
        base = UgvStatus(estop=True, base_online=False)
        base.header.stamp = rospy.Time.from_sec(9.9)
        self.assertEqual(self.server.control_failure(base)[0], NavigateToPointResult.ESTOP)

    def test_mission_blocks_navigation_even_before_the_goal_has_a_grant(self):
        base = UgvStatus(base_online=True, active_velocity_source="mission")
        base.header.stamp = rospy.Time.from_sec(9.9)
        self.assertEqual(self.server.control_failure(base)[0], NavigateToPointResult.HELD)

    def test_external_hold_does_not_credit_the_last_pre_hold_stop(self):
        result = NavigateToPointResult
        handle = Mock()
        task = Task(handle, 1., 120., "old", NavigateToPointGoal())
        base = UgvStatus(base_online=True, hold=True, stop_valid=True,
                         stopped=True, velocity_valid=True)
        base.header.stamp = base.velocity.header.stamp = rospy.Time.from_sec(9.9)
        self.server.base = base
        self.server.begin_interruption(task, result.HELD, "held externally")
        self.server.confirm_stop(task, task.stop_deadline - .1)
        handle.set_aborted.assert_not_called()

        with patch("navigate_server.rospy.Time.now", return_value=rospy.Time.from_sec(10.2)):
            base.header.stamp = base.velocity.header.stamp = rospy.Time.from_sec(10.1)
            self.server.confirm_stop(task, task.stop_deadline - .1)
        terminal, _ = handle.set_aborted.call_args.args
        self.assertEqual(terminal.code, result.HELD)
        self.assertFalse(terminal.stop_commanded)
        self.assertTrue(terminal.stop_valid and terminal.stopped)

    def test_manual_takeover_finishes_with_fresh_nonstop_feedback(self):
        result = NavigateToPointResult
        handle = Mock()
        task = Task(handle, 1., 120., "old", NavigateToPointGoal())
        self.server.begin_interruption(task, result.MANUAL_OVERRIDE, "manual takeover")
        base = UgvStatus(base_online=True, manual_override=True, stop_valid=True,
                         stopped=False, velocity_valid=True)
        base.header.stamp = base.velocity.header.stamp = rospy.Time.from_sec(10.1)
        self.server.base = base
        with patch("navigate_server.rospy.Time.now", return_value=rospy.Time.from_sec(10.2)):
            self.server.confirm_stop(task, task.stop_deadline - .1)
        terminal, _ = handle.set_aborted.call_args.args
        self.assertEqual(terminal.code, result.MANUAL_OVERRIDE)
        self.assertFalse(terminal.stop_commanded)
        self.assertTrue(terminal.stop_valid)
        self.assertFalse(terminal.stopped)


if __name__ == "__main__":
    unittest.main()
