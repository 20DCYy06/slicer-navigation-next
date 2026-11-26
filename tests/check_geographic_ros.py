#!/usr/bin/env python3
"""Exercise preparation, cancellation, aligned WGS84, and stopped GNSS verification."""

from collections import deque
import math

import actionlib
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from slicer_interfaces.msg import NavigateToPointAction, NavigateToPointGoal, NavigateToPointResult, UgvStatus
from slicer_interfaces.srv import Control
from slicer_navigation.observed_space import preparation_candidate

from check_local_ros import Plant, wait_for


HEIGHT_M = 30.0
EARTH_EAST_M = 6378137.0 + HEIGHT_M
EARTH_NORTH_M = 6378137.0 * (1.0 - 0.0066943799901413165) + HEIGHT_M


class GeographicPlant(Plant):
    def __init__(self):
        super().__init__()
        self.fix = rospy.Publisher("/gnss/fix", NavSatFix, queue_size=2)
        self.quality = rospy.Publisher("/gnss/fix_quality", DiagnosticArray, queue_size=2)
        self.source_stamps = deque(maxlen=512)
        self.fix_timer = rospy.Timer(rospy.Duration(0.075), self.publish_fix)

    def publish_fix(self, _event):
        stamp = rospy.Time.now()
        with self.lock:
            east = self.x + 0.065 * math.cos(self.yaw)
            north = self.y + 0.065 * math.sin(self.yaw)
        message = NavSatFix(latitude=math.degrees(north / EARTH_NORTH_M),
                            longitude=math.degrees(east / EARTH_EAST_M), altitude=HEIGHT_M,
                            status=NavSatStatus(status=NavSatStatus.STATUS_GBAS_FIX))
        message.header.stamp, message.header.frame_id = stamp, "gnss_link"
        epoch = f"010000.{stamp.nsecs:09d}"
        values = {"session": "isolated-synthetic", "gga_quality": "4", "fix_valid": "true",
                  "fresh_rtk_usable": "true", "gga_utc_time": epoch,
                  "receiver_gst_utc_time": epoch, "receiver_gst_gga_match": "true",
                  "receiver_gst_lat_std_m": "0.03", "receiver_gst_lon_std_m": "0.03"}
        diagnostic = DiagnosticStatus(name="cup_gnss/receiver",
                                      values=[KeyValue(key, value) for key, value in values.items()])
        quality = DiagnosticArray(status=[diagnostic])
        quality.header.stamp = stamp
        self.source_stamps.append(stamp)
        self.fix.publish(message)
        self.quality.publish(quality)


def geographic_goal(east_m):
    return NavigateToPointGoal(mode=NavigateToPointGoal.WGS84,
                               latitude_deg=0.0, longitude_deg=math.degrees(east_m / EARTH_EAST_M),
                               timeout=rospy.Duration(40), require_geographic_verification=True)


def check(plant):
    snapshots = []
    grids = deque(maxlen=1)
    fused_inputs = deque(maxlen=8)
    rospy.Subscriber("/ugv/status", UgvStatus, lambda value: snapshots.append(value), queue_size=1)
    rospy.Subscriber("/ugv/internal/move_base/global_costmap/costmap", OccupancyGrid,
                     lambda grid: grids.append(grid), queue_size=1)
    rospy.Subscriber("/ugv/localization/gnss_xy", PoseWithCovarianceStamped,
                     lambda pose: fused_inputs.append(pose), queue_size=5)
    action = actionlib.ActionClient("/ugv/navigate", NavigateToPointAction)
    assert action.wait_for_server(rospy.Duration(12)), "public action not online"
    wait_for(lambda: snapshots and snapshots[-1].gnss_valid and snapshots[-1].pose_valid
             and snapshots[-1].base_online and snapshots[-1].stopped, timeout=12)
    assert plant.motion_count == 0, "startup moved before a goal"
    release = rospy.ServiceProxy("/ugv/control/release", Control)
    assert release().accepted, "simulated release rejected"
    wait_for(lambda: not snapshots[-1].hold)
    footprint = rospy.get_param("/ugv/motion/footprint")
    wait_for(lambda: grids and preparation_candidate(grids[-1], snapshots[-1].pose.pose, footprint, 2.0),
             timeout=12)

    feedback = []
    canceled = action.send_goal(geographic_goal(2.65),
                                feedback_cb=lambda _handle, value: feedback.append(value))
    wait_for(lambda: (any(value.phase == value.PREPARING for value in feedback)
                      and plant.motion_count > 0) or canceled.get_result(), timeout=18)
    assert canceled.get_result() is None, canceled.get_result()
    canceled.cancel()
    wait_for(canceled.get_result)
    result = canceled.get_result()
    assert result.code == NavigateToPointResult.CANCELED and result.stop_valid and result.stopped, result
    wait_for(lambda: snapshots[-1].transform_version == 0)

    feedback.clear()
    final = action.send_goal(geographic_goal(2.65),
                             feedback_cb=lambda _handle, value: feedback.append(value))
    wait_for(final.get_result, timeout=43)
    result = final.get_result()
    assert result.code == NavigateToPointResult.SUCCESS, result
    assert result.stop_valid and result.stopped and result.position_error_valid, result
    assert result.error_source == "gnss_xy", result
    assert any(value.projected_target_valid and value.transform_version == 1 for value in feedback)
    assert math.hypot(plant.x - 2.65, plant.y) < 0.30, (plant.x, plant.y)
    if rospy.get_param("/ugv/internal/slicer_navigate/enable_gnss_fusion", False):
        wait_for(lambda: fused_inputs)
        observation = fused_inputs[-1]
        assert observation.header.frame_id == "map"
        assert observation.header.stamp in tuple(plant.source_stamps), observation.header
        assert observation.pose.covariance[0] > 0 and observation.pose.covariance[7] > 0
    else:
        assert not fused_inputs
    print(f"Simulated WGS84 preparation cancel and verified arrival passed ({plant.x:.3f} m final)")


def main():
    rospy.init_node("slicer_geographic_loop_check")
    wait_for(lambda: rospy.has_param("/slicer_lio_projector/base_to_laser")
             and rospy.has_param("/slicer_lio_projector/fast_lio/mapping"))
    plant = GeographicPlant()
    try:
        check(plant)
    finally:
        timers = (plant.fix_timer, plant.timer)
        for timer in timers:
            timer.shutdown()
        for timer in timers:
            timer.join(timeout=1.0)


if __name__ == "__main__":
    main()
