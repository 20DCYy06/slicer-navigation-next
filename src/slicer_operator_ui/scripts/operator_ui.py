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

import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtCore, QtWidgets
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from slicer_interfaces.msg import (
    NavigateToPointAction, NavigateToPointActionFeedback, NavigateToPointFeedback as Feedback,
    NavigateToPointGoal as Goal, NavigateToPointResult as Result, UgvStatus,
)
from slicer_interfaces.srv import Control
from slicer_operator_ui.map_panel import MapPanel
from slicer_operator_ui.services import ServicePanel

try:
    from slicer_visual_grasp.msg import GraspAction, GraspGoal
except ImportError:
    GraspAction = GraspGoal = None


PHASES = {
    Feedback.IDLE: "空闲", Feedback.PREPARING: "配准准备", Feedback.PLANNING: "规划",
    Feedback.NAVIGATING: "导航", Feedback.VERIFYING: "到点复核", Feedback.STOPPING: "停车确认",
}
FIX = {0: "未知", 1: "单点", 2: "Float", 3: "Fixed"}
ALIGNMENT = {0: "未配准", 1: "退化", 2: "可用"}
RESULT_NAMES = {
    getattr(Result, name): name
    for name in (
        "SUCCESS", "CANCELED", "REPLACED", "INVALID_GOAL", "LOCALIZATION_UNAVAILABLE",
        "ALIGNMENT_SPACE_INSUFFICIENT", "OUTSIDE_MAP", "NO_PATH", "MANUAL_OVERRIDE",
        "ESTOP", "BASE_FAULT", "TIMEOUT", "GEOGRAPHIC_VERIFICATION_FAILED",
        "STOP_NOT_CONFIRMED", "HELD",
    )
}


def result_text(action_status, result):
    if result is None:
        return f"未收到导航结果（Action 状态 {action_status}） · 停稳未确认"
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


def target(mode, first, second, altitude, yaw, verify):
    coordinates = tuple(float(value) for value in (first, second))
    if not all(map(math.isfinite, coordinates)):
        raise ValueError("坐标必须为有限数值")
    angle = float(yaw) if yaw.strip() else None
    if angle is not None and not -math.pi <= angle <= math.pi:
        raise ValueError("航向应在 ±π rad 内")
    goal = Goal(has_yaw=angle is not None)
    if mode == "本地 map":
        goal.mode = Goal.LOCAL_POSE
        goal.local_target.header.frame_id = "map"
        goal.local_target.pose.position.x, goal.local_target.pose.position.y = coordinates
        goal.local_target.pose.orientation.z = math.sin((angle or 0.0) / 2.0)
        goal.local_target.pose.orientation.w = math.cos((angle or 0.0) / 2.0)
        return goal, f"map x {coordinates[0]:.3f} m，y {coordinates[1]:.3f} m"
    latitude, longitude = coordinates
    if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
        raise ValueError("纬度需在 ±90° 内，经度需在 ±180° 内")
    goal.mode = Goal.WGS84
    goal.latitude_deg, goal.longitude_deg = coordinates
    goal.require_geographic_verification = verify
    goal.yaw_enu_rad = angle or 0.0
    if altitude.strip():
        goal.altitude_ellipsoid_m = float(altitude)
        if not math.isfinite(goal.altitude_ellipsoid_m):
            raise ValueError("椭球高必须为有限数值")
        goal.has_altitude = True
    height = f"椭球高 {goal.altitude_ellipsoid_m:.2f} m" if goal.has_altitude else "使用会话平面"
    return goal, f"纬度 {latitude:.8f}°，经度 {longitude:.8f}°，{height}"


STYLE = """
QWidget { color: #e6f2f7; background: #101d27; font-size: 15px; }
QFrame[readout="true"] { background: #192d3a; border: 1px solid #345467; border-radius: 8px; }
QFrame[readout="true"] QLabel { background: transparent; }
QLabel[muted="true"] { color: #a7c1d0; font-size: 13px; }
QLabel#SectionTitle { font-size: 17px; font-weight: 600; }
QLabel[tone="ok"] { color: #83e6b3; }
QLabel[tone="warn"] { color: #ffd488; }
QLabel[tone="stop"] { color: #ff9d9c; }
QPushButton, QComboBox, QLineEdit {
    min-height: 36px; padding: 0px 8px; border: 1px solid #395b70;
    border-radius: 6px; background: #203848;
}
QPushButton:hover, QPushButton:checked { background: #2d566d; }
QPushButton[primary="true"] { background: #82d5e9; color: #10242e; font-weight: 600; }
QPushButton[parking="true"] { border: 1px solid #ae6871; color: #ffc2bd; }
QPushButton:disabled { background: #172630; color: #7d94a0; border-color: #293c47; }
QLineEdit:disabled, QCheckBox:disabled { color: #8499a4; }
QTabBar::tab { min-height: 38px; padding: 0px 12px; background: #1a2e3b; }
QTabBar::tab:selected { background: #2b4f62; border-bottom: 2px solid #82d5e9; }
QTabWidget::pane { border: 1px solid #395366; border-radius: 5px; }
QPlainTextEdit { background: #12242f; border: 1px solid #395366; color: #cae2ee; }
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

    def __init__(self, events):
        super().__init__()
        self.events = events
        self.client = None
        self.server_status = None
        self.active = False
        self.cancel_pending = False
        layout = QtWidgets.QGridLayout(self)
        self.state = message("等待视觉节点" if GraspAction else "未加载独立视觉消息包")
        self.details = message()
        layout.addWidget(self.state, 0, 0, 1, 2)
        self.pick_button = button("启动抓取", lambda: self.send(self.pick_button, GraspGoal.PICK), layout, 1, 0)
        self.release_button = button("释放", lambda: self.send(self.release_button, GraspGoal.RELEASE), layout, 1, 1)
        self.cancel_button = button("取消本动作", self.cancel, layout, 2, 0, 2)
        layout.addWidget(self.details, 3, 0, 1, 2)
        layout.setRowStretch(4, 1)
        self.update_actions(False)

    def update_actions(self, base_ready):
        available = self.client is not None and self.server_status is not None and fresh(self.server_status.header.stamp, 1.0)
        remote_busy = available and any(item.status in self.busy_states for item in self.server_status.status_list)
        occupied = self.active or remote_busy
        if GraspAction:
            if not available:
                caption(self.state, "视觉连接中断，终态待确认" if self.active else "等待视觉节点")
            else:
                caption(self.state, "视觉动作执行中" if occupied else "视觉待命")
        for widget in (self.pick_button, self.release_button):
            widget.setEnabled(available and not occupied and base_ready)
        self.cancel_button.setEnabled(self.active and not self.cancel_pending)
        return occupied

    def send(self, widget, command):
        if self.active or not widget.isEnabled():
            return
        self.active, self.cancel_pending = True, False
        caption(self.details, "已发送视觉动作，等待结果")
        self.update_actions(False)
        self.client.send_goal(GraspGoal(command=command),
                              done_cb=lambda state, result: self.events.put(("grasp_result", (state, result))))

    def cancel(self):
        if self.active and not self.cancel_pending:
            self.cancel_pending = True
            self.client.cancel_goal()
            caption(self.details, "已请求取消，等待视觉终态")
            self.update_actions(False)

    def finish(self, state, result):
        self.active, self.cancel_pending = False, False
        caption(self.details, grasp_result_text(state, result))


class Operator(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.events = SimpleQueue()
        self.client = None
        self.nav_server_stamp = rospy.Time()
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
        self.resize(1120, 690)
        self.setStyleSheet(STYLE)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)
        readouts = QtWidgets.QHBoxLayout()
        self.readouts = []
        for name in ("底盘", "RTK", "导航", "线速"):
            box = QtWidgets.QFrame()
            box.setProperty("readout", True)
            items = QtWidgets.QVBoxLayout(box)
            items.setSpacing(1)
            title = message(name)
            title.setProperty("muted", True)
            value = message("等待连接")
            items.addWidget(title)
            items.addWidget(value)
            readouts.addWidget(box, 1)
            self.readouts.append(value)
        layout.addLayout(readouts)

        self.map = MapPanel()
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.addWidget(self.map)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setMinimumWidth(312)
        split.addWidget(self.tabs)
        split.setStretchFactor(0, 2)
        split.setStretchFactor(1, 1)
        split.setCollapsible(0, False)
        split.setCollapsible(1, False)
        layout.addWidget(split, 1)

        nav = QtWidgets.QWidget()
        self.tabs.addTab(nav, "导航")
        form = QtWidgets.QGridLayout(nav)
        form.setColumnStretch(1, 1)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(("本地 map", "WGS84"))
        form.addWidget(self.mode, 0, 0, 1, 2)
        self.fields, self.labels = [], []
        for index in range(4):
            label = message()
            value = QtWidgets.QLineEdit()
            value.setInputMethodHints(QtCore.Qt.ImhFormattedNumbersOnly)
            value.textChanged.connect(self.invalidate_preview)
            self.labels.append(label)
            self.fields.append(value)
            form.addWidget(label, index + 1, 0)
            form.addWidget(value, index + 1, 1)
        self.verify = QtWidgets.QCheckBox("地理到点复核", checked=True)
        self.verify.toggled.connect(self.invalidate_preview)
        form.addWidget(self.verify, 5, 0, 1, 2)
        self.preview_text = message()
        form.addWidget(self.preview_text, 6, 0, 1, 2)
        button("预览目标", self.preview_goal, form, 7, 0)
        self.send_button = button("启动导航", self.send, form, 7, 1)
        self.send_button.setProperty("primary", True)
        self.cancel_button = button("取消本目标", self.cancel, form, 8, 0, 2)
        self.details = message()
        form.addWidget(self.details, 9, 0, 1, 2)
        form.setRowStretch(10, 1)
        self.mode.currentIndexChanged.connect(self.mode_changed)

        rtk = QtWidgets.QWidget()
        readout = QtWidgets.QVBoxLayout(rtk)
        self.rtk_pose = message("等待读取定位")
        self.rtk_receiver = message("等待读取接收机")
        readout.addWidget(self.rtk_pose)
        readout.addWidget(self.rtk_receiver)
        readout.addStretch()
        self.tabs.addTab(rtk, "RTK")
        self.grasp = GraspPanel(self.events)
        self.tabs.addTab(self.grasp, "视觉")
        self.services = ServicePanel(os.environ.get("SLICER_SERVICE_RUN"))
        self.tabs.addTab(self.services, "服务")
        self.tabs.currentChanged.connect(
            lambda index: self.services.enter() if self.tabs.widget(index) is self.services else None
        )

        actions = QtWidgets.QGridLayout()
        self.hold_button = button("驻车 Hold", lambda: self.control("hold"), actions, 0, 0)
        self.hold_button.setProperty("parking", True)
        self.release_button = button("解除 Hold", lambda: self.control("release"), actions, 0, 1)
        self.record_button = button("开始录包", self.record, actions, 0, 2)
        self.notice = message("等待 ROS Master")
        self.record_text = message()
        actions.addWidget(self.notice, 0, 3)
        actions.addWidget(self.record_text, 1, 0, 1, 4)
        if not self.record_executable:
            caption(self.record_text, "录包工具未配置")
        actions.setColumnStretch(3, 1)
        layout.addLayout(actions)
        self.mode_changed()

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
            ]
            if grasp is not None:
                topics.append(("/ugv/grasp/status", GoalStatusArray, "grasp_status"))
            for topic, cls, event in topics:
                subscription = rospy.Subscriber(topic, cls, lambda value, key=event: self.events.put((key, value)),
                                                queue_size=1)
                self.subscribers.append(subscription)
        except (OSError, rospy.ROSException) as error:
            self.events.put(("control", str(error)))

    def connect_map(self):
        self.preview_pub = rospy.Publisher("~target_preview", PoseStamped, queue_size=1, latch=True)
        try:
            self.map.connect_rviz(rospy.resolve_name("~map_point"), self.preview_pub.resolved_name)
        except (ImportError, OSError, RuntimeError) as error:
            caption(self.map.canvas, f"RViz 无法加载 · {error}")

    def update_buttons(self):
        base = self.base
        current = self.ros_ready and base is not None and fresh(base.header.stamp)
        ready = (current and base.base_online
                 and not (base.hold or base.estop or base.manual_override))
        grasp_busy = self.grasp.update_actions(bool(
            ready and not self.active and base.stop_valid and base.stopped
            and base.velocity_valid and fresh(base.velocity.header.stamp, 0.45)
            and not (base.goal_id or base.nav_goal_id) and base.active_velocity_source == "zero"
        ))
        self.send_button.setEnabled(bool(
            self.preview is not None and self.client is not None and fresh(self.nav_server_stamp, 1.0)
            and not self.active and ready and not grasp_busy and base.active_velocity_source != "mission"
        ))
        self.cancel_button.setEnabled(self.active and not self.cancel_pending)
        self.hold_button.setEnabled(self.ros_ready)
        self.release_button.setEnabled(self.ros_ready)
        self.record_button.setEnabled(bool(self.record_executable and self.ros_ready and not self.saving_recording))

    def mode_changed(self, *_args):
        local = self.mode.currentIndex() == 0
        captions = ("x (m)", "y (m)", "椭球高 (m)", "航向 (rad)") if local else (
            "纬度 (°)", "经度 (°)", "椭球高 (m)", "ENU 航向 (rad)"
        )
        for label, text in zip(self.labels, captions):
            caption(label, text)
        for field in self.fields[2:]:
            field.setPlaceholderText("可选")
        self.fields[2].setEnabled(not local)
        self.verify.setEnabled(not local)
        self.invalidate_preview()

    def invalidate_preview(self, *_args):
        self.preview = None
        self.map.show_marker(False)
        caption(self.preview_text, "")
        self.update_buttons()

    def preview_goal(self):
        try:
            self.preview, description = target(
                self.mode.currentText(), *(field.text() for field in self.fields), self.verify.isChecked()
            )
            if self.preview.mode == Goal.WGS84:
                description += " · 投影由服务端反馈"
            else:
                self.publish_preview(self.preview.local_target)
            caption(self.preview_text, description)
        except ValueError as error:
            self.preview = None
            caption(self.preview_text, f"输入无效 · {error}")
        self.update_buttons()

    def map_point(self, pose):
        try:
            x, y = map_coordinates(pose)
        except ValueError as error:
            caption(self.notice, str(error))
            return
        self.tabs.setCurrentIndex(0)
        self.mode.setCurrentIndex(0)
        self.fields[0].setText(f"{x:.4f}")
        self.fields[1].setText(f"{y:.4f}")
        self.fields[3].clear()
        self.preview_goal()
        self.map.select_move()

    def publish_preview(self, pose):
        if self.preview_pub is not None:
            marker = PoseStamped(header=pose.header, pose=pose.pose)
            self.preview_pub.publish(marker)
            self.map.show_marker(True)

    def send(self):
        if not self.send_button.isEnabled():
            return
        self.active, self.cancel_pending = True, False
        self.local_result = False
        self.preview_mode = self.preview.mode
        self.client.send_goal(
            self.preview,
            done_cb=lambda state, result: self.events.put(("result", (state, result))),
            feedback_cb=lambda value: self.events.put(("feedback", value)),
        )
        caption(self.details, "已发送目标，等待服务端")
        self.update_buttons()

    def cancel(self):
        if self.active and not self.cancel_pending:
            self.cancel_pending = True
            self.client.cancel_goal()
            caption(self.details, "已请求取消，等待停车结果")
            self.update_buttons()

    def control(self, name):
        def call():
            try:
                path = "/ugv/control/" + name
                rospy.wait_for_service(path, timeout=1.0)
                response = rospy.ServiceProxy(path, Control)()
                prefix = "已接受" if response.accepted else "已拒绝"
                self.events.put(("control", f"{prefix} · {response.reason}"))
            except (rospy.ROSException, rospy.ServiceException) as error:
                self.events.put(("control", str(error)))
        Thread(target=call, daemon=True).start()

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
        self.record_button.setText("结束录包")
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
        status = self.base
        if status is None or not fresh(status.header.stamp):
            if self.ros_ready:
                caption(self.readouts[0], "状态未连接" if status is None else "状态过期", "warn")
                for readout in self.readouts[1:]:
                    caption(readout, "—", "warn")
                caption(self.rtk_pose, "车端状态不可用")
            return
        if status.estop:
            base, tone = "急停", "stop"
        elif not status.base_online:
            base, tone = "离线", "stop"
        elif status.manual_override:
            base, tone = "人工接管", "warn"
        elif status.hold:
            base, tone = "Hold", "warn"
        else:
            base, tone = "在线", "ok"
        fix = FIX.get(status.gnss_fix_type, "未知") if status.gnss_valid else "无有效观测"
        speed = (f"{status.velocity.twist.linear.x:.2f} m/s"
                 if status.velocity_valid and fresh(status.velocity.header.stamp) else "—")
        caption(self.readouts[0], f"{base} · {stop_label(status.stop_valid, status.stopped)}", tone)
        caption(self.readouts[1], fix, "ok" if fix == "Fixed" else "warn")
        phases = PHASES.get(status.phase, "未知")
        caption(self.readouts[2], phases, "ok" if status.phase == status.IDLE else "warn")
        source = status.active_velocity_source or "—"
        caption(self.readouts[3], f"{speed} · {source}", "ok" if status.velocity_valid else "warn")

        align = ALIGNMENT.get(status.alignment_state, "未知")
        position = status.pose.pose.position
        pose = f"{position.x:.2f}, {position.y:.2f} m" if status.pose_valid else "—"
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
                caption(self.notice, "ROS 已连接")
                self.connect_map()
            elif event == "status":
                self.base = value
            elif event == "navigation_status":
                self.nav_server_stamp = value.header.stamp
            elif event in ("gnss", "sigma", "fix"):
                setattr(self, event, value)
            elif event == "map_point":
                self.map_point(value)
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
                self.record_button.setText("开始录包")
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
        if self.services.operation in ("start", "stop", "restart"):
            self.tabs.setCurrentWidget(self.services)
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
