from pathlib import Path
import sys
import unittest

from actionlib_msgs.msg import GoalStatus
from slicer_interfaces.msg import NavigateToPointResult

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_bringup/scripts"))
from operator_ui import result_text


class OperatorResultTest(unittest.TestCase):
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
