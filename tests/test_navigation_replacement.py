from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_navigation/src"))
from slicer_navigation.goal_ownership import GoalOwnership


class NavigationReplacementTest(unittest.TestCase):
    def test_invalid_goal_leaves_previous_velocity_enabled(self):
        owner = GoalOwnership()
        owner.accept("first", valid=True)
        owner.arm("first", 3, 2)
        assert not owner.accept("invalid", valid=False)
        assert owner.route(4, str, 2) == "first"


    def test_replacement_disables_old_and_late_cancel_cannot_disarm_new(self):
        owner = GoalOwnership()
        owner.accept("first", valid=True)
        owner.arm("first", 3, 2)
        assert owner.accept("second", valid=True)
        assert owner.route(4, str, 2) is None
        assert not owner.arm("first", 4, 2)
        assert owner.arm("second", 6, 5)
        assert not owner.cancel("first")
        assert owner.route(6, str, 5) is None
        assert owner.route(7, str, 5) == "second"


    def test_cancel_while_waiting_for_move_base_blocks_late_activation(self):
        owner = GoalOwnership()
        owner.accept("first", valid=True)
        assert owner.cancel("first")
        assert not owner.arm("first", 3, 2)
        assert owner.route(4, str, 2) is None


    def test_disarming_current_subgoal_discards_its_queued_commands(self):
        owner = GoalOwnership()
        owner.accept("geographic", valid=True)
        owner.arm("geographic", 2, 1)
        assert owner.disarm("geographic")
        assert owner.route(3, str, 1) is None
        assert owner.arm("geographic", 7, 6)
        assert owner.route(6, str, 5) is None
        assert owner.route(8, str, 6) == "geographic"

    def test_controller_command_must_carry_the_current_subgoal_stamp(self):
        owner = GoalOwnership()
        owner.accept("first", valid=True)
        owner.arm("first", 3, target_stamp=2)
        owner.accept("second", valid=True)
        owner.arm("second", 6, target_stamp=5)
        assert owner.route(7, str, target_stamp=2) is None
        assert owner.route(7, str, target_stamp=5) == "second"
