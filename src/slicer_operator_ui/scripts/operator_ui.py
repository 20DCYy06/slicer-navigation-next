#!/usr/bin/env python3
"""Direct field client for the public navigation and visual ROS interfaces."""

import math
import os
from pathlib import Path
from queue import Empty, SimpleQueue
import signal
import subprocess
import sys
from threading import Thread
import time

import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PyQt5 import QtCore, QtWidgets
import rospy
from sensor_msgs.msg import LaserScan, NavSatFix, NavSatStatus

from slicer_interfaces.msg import (
    NavigateToPointAction, NavigateToPointActionFeedback, NavigateToPointFeedback as Feedback,
    NavigateToPointGoal as Goal, NavigateToPointResult as Result, UgvStatus,
)
from slicer_operator_ui.map_panel import MapPanel
from slicer_operator_ui.services import ServicePanel

try:
    from slicer_visual_grasp.msg import GraspAction, GraspGoal
except ImportError:
    GraspAction = GraspGoal = None


PHASES = {
    Feedback.IDLE: "待命", Feedback.PREPARING: "准备定位", Feedback.PLANNING: "规划路线",
    Feedback.NAVIGATING: "行驶中", Feedback.VERIFYING: "确认到点", Feedback.STOPPING: "正在停车",
}
FIX = {0: "未知", 1: "单点", 2: "Float", 3: "Fixed"}
RECEIVER_FIX = {"single": "单点", "differential": "差分", "PPS": "PPS", "RTK float": "Float", "RTK fixed": "Fixed"}
ALIGNMENT = {0: "未配准", 1: "退化", 2: "可用"}
RESULT_NAMES = {
    Result.SUCCESS: "已到达", Result.CANCELED: "已取消", Result.REPLACED: "已换目标",
    Result.INVALID_GOAL: "目标无效", Result.LOCALIZATION_UNAVAILABLE: "定位不可用",
    Result.ALIGNMENT_SPACE_INSUFFICIENT: "校准空间不足", Result.OUTSIDE_MAP: "目标超出地图",
    Result.NO_PATH: "找不到可走路线", Result.MANUAL_OVERRIDE: "已由手柄接管",
    Result.ESTOP: "已急停", Result.BASE_FAULT: "底盘故障", Result.TIMEOUT: "导航超时",
    Result.GEOGRAPHIC_VERIFICATION_FAILED: "未能确认到达", Result.STOP_NOT_CONFIRMED: "停稳未确认",
    Result.HELD: "已驻车",
}


def result_text(_action_status, result):
    if result is None:
        return "未收到导航结果 · 停稳未确认"
    stopped = stop_label(result.stop_valid, result.stopped)
    parts = [str(RESULT_NAMES.get(result.code, result.code))]
    if result.reason:
        parts.append(result.reason)
    parts.append(stopped)
    if result.position_error_valid:
        parts.append(f"位置误差 {result.position_error_m:.2f} m ({result.error_source})")
    return " · ".join(parts)


def grasp_result_text(state, result):
    if result is None or state == GoalStatus.LOST:
        return "视觉终态未知 · 请核对持物与停车状态"
    if state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
        outcome = "已取消"
    elif state == GoalStatus.SUCCEEDED and result.success:
        outcome = "成功"
    else:
        outcome = "失败"
    recovery = "需要释放或恢复" if result.needs_release else "无需释放或恢复"
    return " · ".join(filter(None, (outcome, result.message, recovery)))


def fresh(stamp, limit=0.5):
    return not stamp.is_zero() and 0 <= (rospy.Time.now() - stamp).to_sec() <= limit


def action_transport_ready(client):
    if client is None:
        return False
    transport = client.action_client
    return all(link.get_num_connections() for link in (
        transport.pub_goal, transport.pub_cancel, transport.result_sub, transport.feedback_sub,
    ))


def target(mode, first, second, altitude, verify):
    coordinates = tuple(float(value) for value in (first, second))
    if not all(map(math.isfinite, coordinates)):
        raise ValueError("坐标必须为有限数值")
    goal = Goal()
    if mode == "本地 map":
        goal.mode = Goal.LOCAL_POSE
        goal.local_target.header.frame_id = "map"
        goal.local_target.pose.position.x, goal.local_target.pose.position.y = coordinates
        goal.local_target.pose.orientation.w = 1.0
        return goal, f"目标 X {coordinates[0]:.3f} m，Y {coordinates[1]:.3f} m"
    latitude, longitude = coordinates
    if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
        raise ValueError("纬度需在 ±90° 内，经度需在 ±180° 内")
    goal.mode = Goal.WGS84
    goal.latitude_deg, goal.longitude_deg = coordinates
    goal.require_geographic_verification = verify
    if altitude.strip():
        goal.altitude_ellipsoid_m = float(altitude)
        if not math.isfinite(goal.altitude_ellipsoid_m):
            raise ValueError("椭球高必须为有限数值")
        goal.has_altitude = True
    height = f"，高度 {goal.altitude_ellipsoid_m:.2f} m" if goal.has_altitude else ""
    return goal, f"目标 {latitude:.8f}°，{longitude:.8f}°{height}"


STYLE = """
QWidget { color: #e6f2f7; background: #16212d; font-size: 14px; }
QWidget#OperatorSidebar, QWidget#OperatorSidebarContent { background: #edf0f2; }
QScrollArea#LiveReadouts, QWidget#ReadoutCard { background: #182635; border-radius: 6px; }
QWidget#ReadoutCard QLabel { background: transparent; font-size: 13px; }
QLabel#RtkHeadline { background: transparent; color: #804b15; font-size: 16px; font-weight: 600; }
QLabel#RtkHeadline[tone="ok"] { color: #16654c; }
QLabel#RtkHeadline[tone="stop"] { color: #a02e45; }
QLabel#RtkFreshness, QLabel#OperatorNotice { background: transparent; color: #354b59; }
QLabel[muted="true"] { color: #a7c1d0; font-size: 13px; }
QLabel[tone="ok"] { color: #83e6b3; }
QLabel[tone="warn"] { color: #ffd488; }
QLabel[tone="stop"] { color: #ff9d9c; }
QPushButton, QComboBox, QLineEdit {
    min-height: 26px; padding: 0px 7px; border: 1px solid #395b70;
    border-radius: 4px; background: #203848;
}
QPushButton:hover, QPushButton:checked { background: #2d566d; }
QPushButton:disabled { background: #172630; color: #7d94a0; border-color: #293c47; }
QPushButton#TaskAction { background: #85283e; border-color: #b46576; font-weight: 600; }
QPushButton#TaskAction:hover { background: #a0344f; }
QPushButton#TaskAction:disabled { background: #592735; color: #d6aeb9; }
QPushButton#TaskAction[running="true"] { background: #157455; border-color: #62b797; color: #eafff6; }
QPushButton#TaskAction[running="true"]:hover { background: #1c8f69; }
QLineEdit:disabled, QCheckBox:disabled { color: #8499a4; }
QTabBar::tab { min-height: 26px; padding: 0px 12px; background: #1a2e3b; }
QTabBar::tab:selected { background: #2b4f62; border-bottom: 2px solid #82d5e9; }
QTabWidget::pane { border: 1px solid #395366; border-radius: 4px; }
QPlainTextEdit { background: #12242f; border: 1px solid #395366; color: #cae2ee; }
QWidget#ServicePanel, QLabel#WorkflowState, QLabel#WorkflowAircraft, QToolButton#ServiceDiagnostics {
    background: transparent; color: #354b59;
}
QPushButton#WorkflowAction, QPushButton#WorkflowStop {
    min-height: 22px; text-align: left; background: #542d3d; border-color: #745567;
}
QPushButton#WorkflowAction:hover, QPushButton#WorkflowStop:hover { background: #734052; }
QPushButton#WorkflowAction:disabled, QPushButton#WorkflowStop:disabled { background: #dce1e7; color: #526776; }
QLabel#WorkflowState[state="active"] { color: #16654c; }
QLabel#WorkflowState[state="activating"], QLabel#WorkflowState[state="deactivating"] { color: #804b15; }
QLabel#WorkflowState[state="failed"] { color: #a02e45; }
QWidget#TargetForm QComboBox, QWidget#TargetForm QLineEdit { min-height: 22px; }
QScrollBar:vertical { background: #263844; width: 8px; margin: 0; }
QScrollBar::handle:vertical { background: #8295a0; min-height: 18px; border-radius: 3px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
"""


def caption(label, value, tone=None):
    if label.text() != value:
        label.setText(value)
    if tone is not None and label.property("tone") != tone:
        label.setProperty("tone", tone)
        label.style().unpolish(label)
        label.style().polish(label)


def message(text=""):
    label = QtWidgets.QLabel(text, wordWrap=True)
    label.setTextFormat(QtCore.Qt.PlainText)
    return label


def button(label, callback, grid, row, column, span=1):
    widget = QtWidgets.QPushButton(label)
    widget.clicked.connect(callback)
    grid.addWidget(widget, row, column, 1, span)
    return widget


def task_running(widget, running):
    if widget.property("running") != running:
        widget.setProperty("running", running)
        widget.style().unpolish(widget)
        widget.style().polish(widget)


def stop_label(valid, stopped):
    if not valid:
        return "停稳未确认"
    return "已停稳" if stopped else "仍在运动"


def receiver_fields(array):
    for item in array.status:
        if item.name == "cup_gnss/receiver":
            return item.message, {entry.key: entry.value for entry in item.values}
    return "", {}


def value_text(value):
    return "—" if value in ("", None, "unknown") else str(value)


def map_coordinates(pose):
    x, y = pose.pose.position.x, pose.pose.position.y
    if pose.header.frame_id != "map" or not all(map(math.isfinite, (x, y))):
        raise ValueError("地图选点需要 map 坐标")
    return x, y


def progress_text(feedback):
    phase = PHASES.get(feedback.phase, "未知")
    distance = f" · 剩余 {feedback.remaining_distance_m:.2f} m" if feedback.remaining_distance_valid else ""
    return " · ".join(filter(None, (phase + distance, feedback.reason)))


class GraspPanel(QtWidgets.QWidget):
    busy_states = {GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING}

    def __init__(self, events, start_service):
        super().__init__()
        self.events = events
        self.start_service = start_service
        self.client = self.server_status = None
        self.selected_button = self.command = self.start_deadline = None
        self.cancel_pending = self.base_ready = False
        layout = QtWidgets.QGridLayout(self)
        self.state = message("等待视觉服务" if GraspAction else "视觉功能未安装")
        self.details = message()
        layout.addWidget(self.state, 0, 0, 1, 2)
        self.pick_button = button("抓取", lambda: self.toggle(self.pick_button, GraspGoal.PICK), layout, 1, 0)
        self.release_button = button("释放", lambda: self.toggle(self.release_button, GraspGoal.RELEASE), layout, 1, 1)
        for widget in (self.pick_button, self.release_button):
            widget.setObjectName("TaskAction")
            widget.setToolTip("点击开始，再次点击取消")
        layout.addWidget(self.details, 2, 0, 1, 2)
        layout.setRowStretch(3, 1)
        self.update_actions(False)

    def update_actions(self, base_ready):
        self.base_ready = base_ready
        if self.start_deadline is not None:
            if not base_ready:
                self.start_failed("车辆状态已变化，视觉动作未发送")
            elif self.available() and not self.server_busy():
                self.dispatch()
            elif time.monotonic() >= self.start_deadline:
                self.start_failed("视觉 Action 未就绪，请查看服务日志")
        self.refresh()

    def refresh(self):
        if GraspAction:
            if self.start_deadline is not None:
                caption(self.state, "等待视觉 Action")
            elif not self.available():
                caption(self.state, "视觉连接中断，结果待确认" if self.occupied() else "视觉服务未就绪")
            else:
                caption(self.state, "视觉动作执行中" if self.occupied() else "视觉待命")
        can_start = self.base_ready and self.client is not None and not self.occupied()
        for widget in (self.pick_button, self.release_button):
            running = widget is self.selected_button and not self.cancel_pending
            task_running(widget, running)
            widget.setEnabled(running or can_start)

    def available(self):
        return (self.server_status is not None and fresh(self.server_status.header.stamp, 1.0)
                and action_transport_ready(self.client))

    def server_busy(self):
        return self.available() and any(item.status in self.busy_states for item in self.server_status.status_list)

    def occupied(self):
        return self.selected_button is not None or self.server_busy()

    def toggle(self, widget, command):
        if widget is self.selected_button:
            self.cancel()
            return
        if not widget.isEnabled() or self.occupied():
            return
        self.selected_button, self.command = widget, command
        if self.available():
            self.dispatch()
        else:
            self.start_deadline = time.monotonic() + 15
            if self.start_service():
                caption(self.details, "正在启动视觉服务")
            else:
                self.start_failed("视觉服务无法启动，请查看服务诊断")
        self.refresh()

    def dispatch(self):
        self.start_deadline = None
        caption(self.details, "已发送视觉动作，等待结果")
        self.client.send_goal(GraspGoal(command=self.command),
                              done_cb=lambda state, result: self.events.put(("grasp_result", (state, result))))

    def start_failed(self, reason):
        if self.start_deadline is not None:
            self.reset()
            caption(self.details, reason)
            self.refresh()

    def cancel(self):
        if self.start_deadline is not None:
            self.start_failed("已取消，视觉动作未发送")
        elif self.selected_button is not None and not self.cancel_pending:
            self.cancel_pending = True
            self.client.cancel_goal()
            caption(self.details, "已请求取消，等待视觉终态")
            self.refresh()

    def reset(self):
        self.selected_button = self.command = self.start_deadline = None
        self.cancel_pending = False

    def finish(self, state, result):
        self.reset()
        caption(self.details, grasp_result_text(state, result))
        self.refresh()


class Operator(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.events = SimpleQueue()
        self.client = None
        self.nav_server_stamp = rospy.Time()
        self.nav_stopped_at = rospy.Time()
        self.active = self.cancel_pending = self.closed = False
        self.preview = None
        self.preview_mode = None
        self.preview_pub = None
        self.recorder = None
        self.saving_recording = False
        self.recording_active = False
        self.record_executable = os.environ.get("SLICER_RECORD_RUN")
        self.base = None
        self.gnss = self.sigma = self.fix = None
        self.scan_stamp = self.lio_stamp = rospy.Time()
        self.last_displayed_result = ""
        self.local_result = False
        self.record_path = ""
        self.ros_ready = False
        self.build()
        self.timer = QtCore.QTimer(self, interval=100)
        self.timer.timeout.connect(self.drain_events)
        self.timer.start()
        Thread(target=self.connect_ros, daemon=True).start()

    def build(self):
        self.setWindowTitle("Slicer 现场操作")
        self.setStyleSheet(STYLE)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.map = MapPanel()
        self.map.close_requested.connect(self.close)
        layout.addWidget(self.map, 1)

        content = QtWidgets.QWidget(objectName="OperatorSidebarContent")
        self.sidebar = self.scroll_panel(content)
        self.sidebar.setObjectName("OperatorSidebar")
        layout.addWidget(self.sidebar)
        side = QtWidgets.QVBoxLayout(content)
        side.setContentsMargins(9, 7, 9, 7)
        side.setSpacing(5)
        self.build_readouts(side)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setMinimumHeight(108)
        self.tabs.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Ignored)
        side.addWidget(self.tabs, 1)
        self.build_target_tab()

        rtk = QtWidgets.QWidget()
        readout = QtWidgets.QVBoxLayout(rtk)
        readout.setContentsMargins(6, 5, 6, 5)
        self.rtk_pose = message("等待读取定位")
        self.rtk_receiver = message("等待读取接收机")
        readout.addWidget(self.rtk_pose)
        readout.addWidget(self.rtk_receiver)
        readout.addStretch()
        self.tabs.addTab(self.scroll_panel(rtk), "RTK")
        self.grasp = GraspPanel(self.events, lambda: self.services.ensure_vision())
        self.tabs.addTab(self.scroll_panel(self.grasp), "抓取")

        actions = QtWidgets.QGridLayout()
        actions.setSpacing(4)
        self.navigation_button = button("导航", self.toggle_navigation, actions, 0, 0)
        self.navigation_button.setObjectName("TaskAction")
        self.navigation_button.setToolTip("导航到已选目标，再次点击取消")
        self.record_button = button("录包", self.record, actions, 0, 1)
        self.notice = message("等待车端服务")
        self.record_text = message()
        for row, widget in enumerate((self.notice, self.record_text), start=1):
            widget.setObjectName("OperatorNotice")
            actions.addWidget(widget, row, 0, 1, 2)
        self.record_text.hide()
        side.addLayout(actions)

        self.services = ServicePanel(os.environ.get("SLICER_SERVICE_RUN"))
        self.services.stop_requested.connect(self.stop_services)
        self.services.navigation_stopped.connect(self.navigation_stopped)
        self.services.vision_start_failed.connect(self.grasp.start_failed)
        side.addWidget(self.services)
        self.tabs.currentChanged.connect(self.update_buttons)
        self.mode_changed()
        self.resize(1280, 720)
        self.sidebar.setFixedWidth(self.sidebar_width())

    def sidebar_width(self):
        return max(284, self.width() // 4)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.sidebar.setFixedWidth(self.sidebar_width())

    @staticmethod
    def scroll_panel(content):
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)
        return scroll

    def build_readouts(self, sidebar):
        self.rtk_headline = message("RTK状态 · 等待数据")
        self.rtk_headline.setObjectName("RtkHeadline")
        self.rtk_freshness = message("等待新鲜 RTK  |  map")
        self.rtk_freshness.setObjectName("RtkFreshness")
        sidebar.addWidget(self.rtk_headline)
        sidebar.addWidget(self.rtk_freshness)

        card = QtWidgets.QWidget(objectName="ReadoutCard")
        lines = QtWidgets.QVBoxLayout(card)
        lines.setContentsMargins(8, 5, 8, 5)
        lines.setSpacing(2)
        self.position_text = message("当前位置 · —")
        self.waypoint_text = message("Waypoint · 等待打点")
        self.alignment_text = message("配准 · 等待状态")
        self.mission_text = message("任务阶段 · 等待状态")
        self.scan_text = message("MID360 扫描 · 等待新数据")
        self.lio_text = message("LIO · 等待新数据")
        self.base_text = message("底盘驱动 · 等待状态")
        self.speed_text = message("速度 · —")
        for label in (self.alignment_text, self.mission_text, self.scan_text, self.lio_text,
                      self.base_text, self.position_text, self.waypoint_text, self.speed_text):
            lines.addWidget(label)
        scroll = self.scroll_panel(card)
        scroll.setObjectName("LiveReadouts")
        scroll.setMinimumHeight(112)
        scroll.setMaximumHeight(126)
        sidebar.addWidget(scroll)

    def build_target_tab(self):
        nav = QtWidgets.QWidget(objectName="TargetForm")
        self.tabs.addTab(self.scroll_panel(nav), "导航")
        form = QtWidgets.QGridLayout(nav)
        form.setContentsMargins(6, 5, 6, 5)
        form.setSpacing(3)
        form.setColumnStretch(1, 1)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem("地图坐标", "本地 map")
        self.mode.addItem("经纬度", "WGS84")
        form.addWidget(self.mode, 0, 0, 1, 2)
        self.fields, self.labels = [], []
        for index in range(3):
            label = message()
            value = QtWidgets.QLineEdit()
            value.setInputMethodHints(QtCore.Qt.ImhFormattedNumbersOnly)
            value.textChanged.connect(self.preview_goal)
            self.labels.append(label)
            self.fields.append(value)
            form.addWidget(label, index + 1, 0)
            form.addWidget(value, index + 1, 1)
        self.verify = QtWidgets.QCheckBox("到点后用 RTK 确认", checked=True)
        self.verify.toggled.connect(self.preview_goal)
        form.addWidget(self.verify, 4, 0, 1, 2)
        self.preview_text = message()
        form.addWidget(self.preview_text, 5, 0, 1, 2)
        self.drive_hint = message()
        self.drive_hint.setProperty("muted", True)
        form.addWidget(self.drive_hint, 6, 0, 1, 2)
        self.details = message()
        form.addWidget(self.details, 7, 0, 1, 2)
        form.setRowStretch(8, 1)
        self.mode.currentIndexChanged.connect(self.mode_changed)

    def connect_ros(self):
        try:
            # The service panel remains available while the ROS Master is down.
            rospy.init_node("slicer_operator_ui", disable_signals=True)
            nav = actionlib.SimpleActionClient("/ugv/navigate", NavigateToPointAction)
            grasp = actionlib.SimpleActionClient("/ugv/grasp", GraspAction) if GraspAction else None
            self.events.put(("ros", (nav, grasp)))
            self.subscribers = []
            topics = [
                ("/ugv/status", UgvStatus, "status"),
                ("/ugv/navigate/status", GoalStatusArray, "navigation_status"),
                ("/ugv/navigate/feedback", NavigateToPointActionFeedback, "other_feedback"),
                ("/gnss/diagnostics", DiagnosticArray, "gnss"),
                ("/gnss/fix_quality", DiagnosticArray, "sigma"),
                ("/gnss/fix", NavSatFix, "fix"),
                ("~map_point", PoseStamped, "map_point"),
                ("~waypoint_point", PoseStamped, "waypoint_point"),
            ]
            if grasp is not None:
                topics.append(("/ugv/grasp/status", GoalStatusArray, "grasp_status"))
            for topic, cls, event in topics:
                subscription = rospy.Subscriber(topic, cls, lambda value, key=event: self.events.put((key, value)),
                                                queue_size=1)
                self.subscribers.append(subscription)
            for topic, cls, event in (("/scan", LaserScan, "scan_stamp"), ("/lio/odom", Odometry, "lio_stamp")):
                subscription = rospy.Subscriber(
                    topic, cls, lambda value, key=event: self.events.put((key, value.header.stamp)), queue_size=1
                )
                self.subscribers.append(subscription)
        except (OSError, rospy.ROSException) as error:
            self.events.put(("control", str(error)))

    def connect_map(self):
        self.preview_pub = rospy.Publisher("~target_preview", PoseStamped, queue_size=1, latch=True)
        try:
            self.map.connect_rviz(rospy.resolve_name("~map_point"), self.preview_pub.resolved_name,
                                  rospy.resolve_name("~waypoint_point"))
            if self.preview is not None and self.preview.mode == Goal.LOCAL_POSE:
                self.publish_preview(self.preview.local_target)
        except (ImportError, OSError, RuntimeError) as error:
            caption(self.map.canvas, f"RViz 无法加载 · {error}")

    def update_buttons(self, *_args):
        drive_block = self.drive_blocked()
        stopping = self.services.stopping_work
        self.grasp.update_actions(not self.active and not drive_block and not stopping and self.stationary())
        nav_available = fresh(self.nav_server_stamp, 1.0) and action_transport_ready(self.client)
        self.services.set_navigation_ready(nav_available)
        if not nav_available:
            nav_block = "请先点「启动导航」"
        elif stopping:
            nav_block = "正在停止作业"
        elif self.grasp.occupied() or (self.base and self.base.active_velocity_source == "mission"):
            nav_block = "视觉或其他任务正在用车"
        else:
            nav_block = drive_block
        running = self.active and not self.cancel_pending
        ready = self.tabs.currentIndex() == 0 and not self.active and self.preview is not None and not nav_block
        task_running(self.navigation_button, running)
        self.navigation_button.setEnabled(running or ready)
        caption(self.drive_hint, nav_block if not self.active else "")
        self.record_button.setEnabled(bool(self.record_executable and self.ros_ready and not self.saving_recording))

    def drive_blocked(self):
        base = self.base
        if base is None or not fresh(base.header.stamp):
            return "正在等车辆状态"
        if base.estop:
            return "车辆急停中"
        if not base.base_online:
            return "底盘未就绪"
        if base.hold:
            return "车端已驻车"
        if base.manual_override or base.active_velocity_source == "teleop":
            return "手柄正在接管"
        return ""

    def stationary(self):
        base = self.base
        return (base is not None and base.stop_valid and base.stopped and base.velocity_valid
                and fresh(base.velocity.header.stamp, .45) and base.active_velocity_source == "zero"
                and not (base.goal_id or base.nav_goal_id))

    def mode_changed(self, *_args):
        local = self.mode.currentIndex() == 0
        captions = ("X (m)", "Y (m)", "高度 (m)") if local else (
            "纬度 (°)", "经度 (°)", "椭球高 (m)"
        )
        for label, text in zip(self.labels, captions):
            caption(label, text)
        self.fields[2].setPlaceholderText("可选")
        self.labels[2].setVisible(not local)
        self.fields[2].setVisible(not local)
        self.verify.setVisible(not local)
        self.preview_goal()

    def preview_goal(self, *_args):
        self.preview = None
        self.map.show_marker(False)
        values = [field.text() for field in self.fields]
        if all(value.strip() for value in values[:2]):
            try:
                self.preview, description = target(self.mode.currentData(), *values, self.verify.isChecked())
                if self.preview.mode == Goal.LOCAL_POSE:
                    self.publish_preview(self.preview.local_target)
                caption(self.preview_text, description)
            except ValueError as error:
                caption(self.preview_text, str(error))
        else:
            caption(self.preview_text, "用 2D Nav Goal 选点，或直接输入坐标" if self.mode.currentIndex() == 0
                    else "输入目标的纬度和经度")
        self.update_buttons()

    def map_point(self, pose):
        try:
            x, y = map_coordinates(pose)
        except ValueError as error:
            caption(self.notice, str(error))
            return
        if not fresh(pose.header.stamp, 1.0):
            caption(self.notice, "目标已过期，请重新选点")
            return
        if self.active:
            caption(self.notice, "等待当前导航结束后再选点")
            self.map.select_move()
            return
        self.tabs.setCurrentIndex(0)
        self.mode.setCurrentIndex(0)
        self.fields[0].setText(f"{x:.4f}")
        self.fields[1].setText(f"{y:.4f}")
        self.preview_goal()
        self.map.select_move()
        self.toggle_navigation()

    def waypoint_point(self, pose):
        try:
            x, y = map_coordinates(pose)
        except ValueError as error:
            caption(self.notice, str(error))
            return
        coordinates = f"map X {x:.3f} m，Y {y:.3f} m"
        caption(self.waypoint_text, f"Waypoint · {coordinates}")
        caption(self.notice, f"Waypoint · {coordinates}")

    def publish_preview(self, pose):
        if self.preview_pub is not None:
            marker = PoseStamped(header=pose.header, pose=pose.pose)
            self.preview_pub.publish(marker)
            self.map.show_marker(True)

    def toggle_navigation(self):
        if self.active:
            self.cancel()
            return
        self.update_buttons()
        if not self.navigation_button.isEnabled():
            return
        goal = self.preview
        self.active, self.cancel_pending = True, False
        self.local_result = False
        self.preview_mode = goal.mode
        self.client.send_goal(
            goal,
            done_cb=lambda state, result: self.events.put(("result", (state, result))),
            feedback_cb=lambda value: self.events.put(("feedback", value)),
        )
        caption(self.details, "已发送导航目标")
        self.update_buttons()

    def cancel(self):
        if self.active and not self.cancel_pending:
            self.cancel_pending = True
            self.client.cancel_goal()
            caption(self.details, "已请求取消，等待导航终态")
            self.update_buttons()

    def stop_services(self):
        self.cancel()
        self.grasp.cancel()
        self.services.stop_work()
        self.update_buttons()

    def navigation_stopped(self):
        if self.ros_ready:
            self.nav_stopped_at = rospy.Time.now()
        self.nav_server_stamp = rospy.Time()
        self.update_buttons()

    def record(self):
        if self.recorder is not None:
            self.stop_record()
            self.saving_recording = True
            caption(self.record_text, "正在保存录包")
            return
        try:
            self.recorder = subprocess.Popen(
                [self.record_executable], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True,
            )
        except OSError as error:
            caption(self.record_text, str(error))
            return
        self.record_button.setText("停止录包")
        caption(self.record_text, "正在启动录包")
        Thread(target=self.read_record_output, args=(self.recorder,), daemon=True).start()

    def stop_record(self):
        if self.recorder is not None and self.recorder.poll() is None:
            try:
                os.killpg(self.recorder.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def read_record_output(self, process):
        with process.stdout as stream:
            for line in stream:
                if line.startswith("/"):
                    self.events.put(("record_path", line.rstrip()))
                elif line.strip():
                    self.events.put(("record_line", line.strip()))
        self.events.put(("recorded", process.wait()))

    def show_status(self):
        for label, name, stamp in ((self.scan_text, "MID360 扫描", self.scan_stamp),
                                   (self.lio_text, "LIO", self.lio_stamp)):
            ready = fresh(stamp, 1.0)
            caption(label, f"{name} · 就绪" if ready else f"{name} · 等待新数据", "ok" if ready else "warn")

        status = self.base
        if status is None or not fresh(status.header.stamp):
            if self.ros_ready:
                reason = "状态未连接" if status is None else "状态过期"
                caption(self.base_text, f"底盘驱动 · {reason}", "warn")
                caption(self.alignment_text, "配准 · 等待状态", "warn")
                caption(self.mission_text, "任务阶段 · 等待状态", "warn")
                caption(self.position_text, "当前位置 · —")
                caption(self.speed_text, "速度 · —", "warn")
                caption(self.rtk_pose, "车端状态不可用")
            return
        if status.estop:
            base, tone = "急停", "stop"
        elif not status.base_online:
            base, tone = "离线", "stop"
        elif status.manual_override:
            base, tone = "人工接管", "warn"
        elif status.hold:
            base, tone = "驻车", "warn"
        else:
            base, tone = "在线", "ok"
        speed = (f"{status.velocity.twist.linear.x:.2f} m/s"
                 if status.velocity_valid and fresh(status.velocity.header.stamp) else "—")
        caption(self.base_text, f"底盘驱动 · {base} · {stop_label(status.stop_valid, status.stopped)}", tone)
        phases = PHASES.get(status.phase, "未知")
        caption(self.mission_text, f"任务阶段 · {phases}", "ok" if status.phase == status.IDLE else "warn")
        source = status.active_velocity_source or "—"
        caption(self.speed_text, f"速度 · {speed} · {source}", "ok" if speed != "—" else "warn")

        align = ALIGNMENT.get(status.alignment_state, "未知")
        caption(self.alignment_text, f"配准 · {align}", "ok" if status.alignment_state == status.ALIGNMENT_READY else "warn")
        position = status.pose.pose.position
        pose = f"{position.x:.2f}, {position.y:.2f} m" if status.pose_valid else "—"
        caption(self.position_text, f"当前位置 · {pose}")
        wgs84 = (f"{status.wgs84_latitude_deg:.8f}°, {status.wgs84_longitude_deg:.8f}°"
                 if status.wgs84_valid else "—")
        yaw = f"{math.degrees(status.yaw_enu_rad):.1f}°" if status.yaw_enu_valid else "—"
        observed_age = observed_time(status.gnss_stamp) if status.gnss_valid else "—"
        caption(self.rtk_pose, f"配准 {align} · 版本 {status.transform_version}\n"
                f"地理观测 {observed_age}\nmap {pose}\n车体 WGS84 {wgs84}\nENU 航向 {yaw}\n"
                f"{status.geographic_reason}")
        if (not self.active and not self.local_result and status.last_result_valid and status.navigation_reason
                and status.last_goal_id != self.last_displayed_result):
            result = RESULT_NAMES.get(status.last_result_code, status.last_result_code)
            caption(self.details, f"{result} · {status.navigation_reason}")
            self.last_displayed_result = status.last_goal_id

    def show_receiver(self):
        info, fields = receiver_fields(self.gnss) if self.gnss and fresh(self.gnss.header.stamp, 2.5) else ("", {})
        quality = {}
        if (self.sigma and self.fix and self.sigma.header.stamp == self.fix.header.stamp
                and fresh(self.fix.header.stamp, 2.5)):
            _, quality = receiver_fields(self.sigma)
        fix_type, tone = "等待数据", "warn"
        status = self.base
        valid = (status is not None and fresh(status.header.stamp) and status.gnss_valid
                 and fresh(status.gnss_stamp, 2.5))
        if valid:
            fix_type = FIX.get(status.gnss_fix_type, "未知")
        elif fields.get("fix_valid") == "true":
            fix_type = RECEIVER_FIX.get(fields.get("fix_type"), "未知")
        elif self.fix and fresh(self.fix.header.stamp, 2.5) and self.fix.status.status == NavSatStatus.STATUS_NO_FIX:
            fix_type, tone = "NO FIX", "stop"
        if fix_type == "Fixed":
            tone = "ok"
        caption(self.rtk_headline, f"RTK状态 · {fix_type}（卫星 {value_text(fields.get('satellites'))}）", tone)
        usable = quality.get("fresh_rtk_usable") == "true"
        caption(self.rtk_freshness, "新鲜 RTK 可用  |  map" if usable else "等待新鲜 RTK  |  map")

        correction = fields.get("rtcm_serial_write_age_s")
        correction = f"{correction} s" if value_text(correction) != "—" else "—"
        ntrip = ("已连接" if fields.get("ntrip_connected") == "true"
                 else "未连接" if fields.get("ntrip_enabled") == "true" else "—")
        geodetic = "—"
        if self.fix and fresh(self.fix.header.stamp, 2.5) and self.fix.status.status != NavSatStatus.STATUS_NO_FIX:
            geodetic = f"{self.fix.latitude:.8f}°, {self.fix.longitude:.8f}°"
        sigma_e = value_text(quality.get("receiver_gst_lon_std_m"))
        sigma_n = value_text(quality.get("receiver_gst_lat_std_m"))
        caption(self.rtk_receiver, f"接收机 {info or '—'}\n"
                f"卫星 {value_text(fields.get('satellites'))} · HDOP {value_text(fields.get('hdop'))}\n"
                f"NTRIP {ntrip} · 差分 {correction}\n天线 WGS84 {geodetic}\n"
                f"同历元 GST 1σ  东 {sigma_e} m · 北 {sigma_n} m")

    def drain_events(self):
        while True:
            try:
                event, value = self.events.get_nowait()
            except Empty:
                break
            if event == "ros":
                self.client, self.grasp.client = value
                self.ros_ready = True
                caption(self.notice, "已连接车端")
                self.connect_map()
            elif event == "status":
                self.base = value
            elif event == "navigation_status":
                if value.header.stamp > self.nav_stopped_at:
                    self.nav_server_stamp = value.header.stamp
            elif event in ("gnss", "sigma", "fix", "scan_stamp", "lio_stamp"):
                setattr(self, event, value)
            elif event == "map_point":
                self.map_point(value)
            elif event == "waypoint_point":
                self.waypoint_point(value)
            elif event == "feedback":
                caption(self.details, progress_text(value))
                if value.projected_target_valid:
                    point = value.projected_target.pose.position
                    self.publish_preview(value.projected_target)
                    caption(self.preview_text, f"map ({point.x:.3f}, {point.y:.3f}) m · 变换 {value.transform_version}")
                elif self.preview_mode == Goal.WGS84:
                    caption(self.preview_text, "投影位置待配准确定")
            elif event == "other_feedback":
                if (not self.active and self.base and self.base.goal_id
                        and value.status.goal_id.id == self.base.goal_id and fresh(value.header.stamp)):
                    self.local_result = False
                    caption(self.details, progress_text(value.feedback))
            elif event == "result":
                self.active = self.cancel_pending = False
                self.local_result = True
                caption(self.details, result_text(*value))
                if self.base is not None:
                    self.last_displayed_result = self.base.last_goal_id
            elif event == "grasp_status":
                self.grasp.server_status = value
            elif event == "grasp_result":
                self.grasp.finish(*value)
            elif event == "control":
                caption(self.notice, value)
            elif event == "record_path":
                self.record_path = value
                caption(self.record_text, f"正在启动录包 · {value}")
            elif event == "record_line" and not self.record_path:
                caption(self.record_text, value)
            elif event == "recorded":
                self.recorder = None
                self.saving_recording = False
                self.recording_active = False
                self.record_button.setText("录包")
                saved = self.record_path and any(Path(self.record_path).glob("*.bag"))
                if value in (0, -signal.SIGINT, 130):
                    result = "录包已结束" if saved else "录包未生成"
                else:
                    result = f"录包退出码 {value}"
                caption(self.record_text, " · ".join(filter(None, (result, self.record_path))))
                self.record_path = ""
        if (self.record_path and not self.recording_active and not self.saving_recording
                and any(Path(self.record_path).glob("*.bag.active"))):
            self.recording_active = True
            caption(self.record_text, f"正在录包 · {self.record_path}")
        self.record_text.setVisible(bool(self.record_text.text()))
        self.show_status()
        self.show_receiver()
        self.update_buttons()
        if rospy.is_shutdown():
            QtWidgets.QApplication.quit()

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        self.timer.stop()
        self.cancel()
        self.grasp.cancel()
        self.stop_record()
        self.map.close_map()

    def closeEvent(self, event):
        if self.services.stopping_work or self.services.operation in ("start", "stop", "restart"):
            event.ignore()
            return
        self.shutdown()
        event.accept()


def observed_time(stamp):
    age = (rospy.Time.now() - stamp).to_sec()
    return f"{age:.1f} s 前" if not stamp.is_zero() and age >= 0.0 else "—"


def main():
    app = QtWidgets.QApplication(sys.argv[:1])
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())
    window = Operator()
    window.showMaximized()
    try:
        return app.exec_()
    finally:
        window.shutdown()
        rospy.signal_shutdown("Operator UI closed")


if __name__ == "__main__":
    raise SystemExit(main())
