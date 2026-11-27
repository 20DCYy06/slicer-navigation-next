#!/usr/bin/env python3
"""Thin field client for the public ROS action, state and control services."""
import math
import os
import signal
import subprocess
from threading import Thread
from queue import Empty, SimpleQueue
import tkinter as tk
from tkinter import ttk

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from slicer_interfaces.msg import (
    NavigateToPointAction, NavigateToPointFeedback as Feedback,
    NavigateToPointGoal as Goal, NavigateToPointResult as Result, UgvStatus,
)
from slicer_interfaces.srv import Control

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
    stopped = "已停稳" if result.stop_valid and result.stopped else "停稳未确认"
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


class GraspPanel(ttk.LabelFrame):
    busy_states = {GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING}

    def __init__(self, parent, events):
        super().__init__(parent, text="视觉伺服", padding=6)
        self.client = actionlib.SimpleActionClient("/ugv/grasp", GraspAction) if GraspAction else None
        self.events = events
        self.connected = False
        self.server_status = None
        self.active = False
        self.cancel_pending = False
        self.state = tk.StringVar(value="等待视觉节点" if self.client else "未加载视觉消息包")
        self.details = tk.StringVar()
        self.pick_button = ttk.Button(self, text="启动抓取", command=lambda: self.send(self.pick_button, GraspGoal.PICK))
        self.release_button = ttk.Button(self, text="释放", command=lambda: self.send(self.release_button, GraspGoal.RELEASE))
        self.cancel_button = ttk.Button(self, text="取消本动作", command=self.cancel)
        for column, button in enumerate((self.pick_button, self.release_button, self.cancel_button)):
            button.grid(row=0, column=column, sticky="w", padx=3)
            button.state(["disabled"])
        for row, variable in enumerate((self.state, self.details), 1):
            ttk.Label(self, textvariable=variable, wraplength=590, justify="left").grid(
                row=row, column=0, columnspan=3, sticky="w", pady=2
            )
        if self.client is not None:
            self.subscriber = rospy.Subscriber(
                "/ugv/grasp/status", GoalStatusArray,
                lambda message: events.put(("grasp_status", message)), queue_size=1,
            )
            Thread(target=self.wait_for_action, daemon=True).start()

    def wait_for_action(self):
        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration(1.0)):
                self.events.put(("grasp_connected", None))
                return

    def update(self, base_ready):
        available = self.connected and self.server_status is not None and fresh(self.server_status.header.stamp, 1.0)
        remote_busy = available and any(item.status in self.busy_states for item in self.server_status.status_list)
        occupied = self.active or remote_busy
        if self.client is not None:
            if not available:
                self.state.set("视觉连接中断，终态待确认" if self.active else "等待视觉节点")
            else:
                self.state.set("视觉动作执行中" if occupied else "视觉待命")
        for button in (self.pick_button, self.release_button):
            button.state(["!disabled" if available and not occupied and base_ready else "disabled"])
        self.cancel_button.state(["!disabled" if self.active and not self.cancel_pending else "disabled"])
        return occupied

    def send(self, button, command):
        if self.active or not button.instate(["!disabled"]):
            return
        self.active, self.cancel_pending = True, False
        self.details.set("已发送视觉动作，等待结果")
        self.update(False)
        self.client.send_goal(GraspGoal(command=command),
                              done_cb=lambda state, result: self.events.put(("grasp_result", (state, result))))

    def cancel(self):
        if self.active and not self.cancel_pending:
            self.cancel_pending = True
            self.client.cancel_goal()
            self.details.set("已请求取消，等待视觉终态")
            self.update(False)

    def finish(self, state, result):
        self.active, self.cancel_pending = False, False
        self.details.set(grasp_result_text(state, result))


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


class Operator:
    def __init__(self, root):
        self.root = root
        self.client = actionlib.SimpleActionClient("/ugv/navigate", NavigateToPointAction)
        self.events = SimpleQueue()
        self.connected = False
        self.active = False
        self.preview = None
        self.recorder = None
        self.last_status = rospy.Time()
        self.base = None
        self.motion_available = False
        self.last_displayed_result = ""
        self.record_path = ""
        rospy.Subscriber("/ugv/status", UgvStatus, lambda msg: self.events.put(("status", msg)),
                         queue_size=1)
        self.mode = tk.StringVar(value="本地 map")
        self.verify = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="等待 /ugv/status")
        self.details = tk.StringVar(value="")
        self.preview_text = tk.StringVar(value="")
        self.record_text = tk.StringVar(value="")
        self.values = [tk.StringVar() for _ in range(4)]
        self.labels = [tk.StringVar() for _ in self.values]
        self.build()
        for value in [self.mode, self.verify, *self.values]:
            value.trace_add("write", self.invalidate_preview)
        self.invalidate_preview()
        self.root.after(80, self.drain_events)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        Thread(target=self.wait_for_action, daemon=True).start()

    def build(self):
        self.root.title("Slicer 现场导航")
        panel = ttk.Frame(self.root, padding=16)
        panel.grid(sticky="nsew")
        for row, variable in ((0, self.status), (1, self.details), (10, self.record_text)):
            ttk.Label(panel, textvariable=variable, wraplength=620, justify="left").grid(
                row=row, column=0, columnspan=4, sticky="w", pady=4
            )
        ttk.Combobox(panel, state="readonly", textvariable=self.mode,
                     values=("本地 map", "WGS84"), width=16).grid(row=2, column=0, sticky="w")
        for index, (label, value) in enumerate(zip(self.labels, self.values)):
            row = 3 + index // 2
            column = 2 * (index % 2)
            ttk.Label(panel, textvariable=label).grid(row=row, column=column, sticky="w", pady=3)
            entry = ttk.Entry(panel, textvariable=value, width=20)
            entry.grid(row=row, column=column + 1, sticky="ew", padx=4)
            if index == 2:
                self.altitude_entry = entry
        self.verify_box = ttk.Checkbutton(panel, text="地理到点复核", variable=self.verify)
        self.verify_box.grid(row=5, column=0, columnspan=2, sticky="w")
        ttk.Label(panel, textvariable=self.preview_text, wraplength=620).grid(
            row=6, column=0, columnspan=4, sticky="w", pady=5
        )
        for column, (name, handler) in enumerate((
            ("预览目标", self.preview_goal), ("启动导航", self.send), ("取消本目标", self.cancel),
            ("驻车 Hold", lambda: self.control("hold")),
        )):
            button = ttk.Button(panel, text=name, command=handler)
            button.grid(row=7, column=column, sticky="ew", padx=3, pady=4)
            if column == 1:
                self.send_button = button
            if column == 2:
                self.cancel_button = button
        ttk.Button(panel, text="解除 Hold", command=lambda: self.control("release")).grid(
            row=8, column=0, sticky="ew", padx=3
        )
        self.record_button = ttk.Button(panel, text="开始录包", command=self.record)
        self.record_button.grid(row=8, column=1, sticky="ew", padx=3)
        self.grasp = GraspPanel(panel, self.events)
        self.grasp.grid(row=9, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        self.cancel_button.state(["disabled"])

    def wait_for_action(self):
        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration(1.0)):
                self.events.put(("connected", None))
                return

    def update_buttons(self):
        ready = fresh(self.last_status) and self.motion_available
        base = self.base
        grasp_busy = self.grasp.update(
            ready and not self.active and base is not None and base.stop_valid and base.stopped
            and base.velocity_valid and fresh(base.velocity.header.stamp, 0.45)
            and not base.goal_id and not base.nav_goal_id and base.active_velocity_source == "zero"
        )
        enabled = (
            self.preview is not None and self.connected and not self.active
            and ready and not grasp_busy and base.active_velocity_source != "mission"
        )
        self.send_button.state(["!disabled" if enabled else "disabled"])
        self.cancel_button.state(["!disabled" if self.active else "disabled"])

    def invalidate_preview(self, *_args):
        local = self.mode.get() == "本地 map"
        labels = ("x (m)", "y (m)", "椭球高 (m，可选)", "航向 (rad，可选)") if local else (
            "纬度 (°)", "经度 (°)", "椭球高 (m，可选)", "ENU 航向 (rad，可选)"
        )
        for label, caption in zip(self.labels, labels):
            label.set(caption)
        self.altitude_entry.state(["disabled" if local else "!disabled"])
        self.verify_box.state(["disabled" if local else "!disabled"])
        self.preview = None
        self.preview_text.set("")
        self.update_buttons()

    def preview_goal(self):
        try:
            self.preview, description = target(
                self.mode.get(), *(value.get() for value in self.values), self.verify.get()
            )
            if self.preview.mode == Goal.WGS84:
                description += "。投影位置由服务端反馈"
            self.preview_text.set(description)
        except ValueError as error:
            self.preview = None
            self.preview_text.set(f"输入无效 · {error}")
        self.update_buttons()

    def send(self):
        if self.preview is None or self.active or not self.send_button.instate(["!disabled"]):
            return
        self.active = True
        self.client.send_goal(
            self.preview,
            done_cb=lambda state, result: self.events.put(("result", (state, result))),
            feedback_cb=lambda value: self.events.put(("feedback", value)),
        )
        self.details.set("已发送目标，等待服务端")
        self.update_buttons()

    def cancel(self):
        if self.active:
            self.client.cancel_goal()
            self.details.set("已请求取消，等待停车结果")

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
            if self.recorder.poll() is None:
                os.killpg(self.recorder.pid, signal.SIGINT)
                self.record_button.state(["disabled"])
                self.record_text.set("正在保存录包")
            return
        try:
            self.recorder = subprocess.Popen(
                ["rosrun", "slicer_bringup", "record_run.py"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True,
            )
        except OSError as error:
            self.record_text.set(str(error))
            return
        self.record_button.configure(text="结束录包")
        self.record_text.set("正在启动录包")
        Thread(target=self.read_record_output, args=(self.recorder,), daemon=True).start()

    def read_record_output(self, process):
        for line in process.stdout:
            if line.startswith("/"):
                self.events.put(("recording", line.rstrip()))
        self.events.put(("recorded", process.wait()))

    def show_status(self, status):
        self.base = status
        self.last_status = status.header.stamp
        blocked = status.hold or status.estop or status.manual_override
        self.motion_available = status.base_online and not blocked
        fix = FIX.get(status.gnss_fix_type, "未知") if status.gnss_valid else "无有效观测"
        stop = "已停稳" if status.stopped else "未停稳" if status.stop_valid else "停稳未知"
        if not status.stop_valid:
            stop = "停稳未知"
        base = "急停" if status.estop else "人工接管" if status.manual_override else (
            "Hold" if status.hold else "在线" if status.base_online else "离线"
        )
        pose = status.pose.pose.position
        position = f"map ({pose.x:.2f}, {pose.y:.2f}) m" if status.pose_valid else "位姿无效"
        self.status.set(
            f"底盘 {base} · {stop} · GNSS {fix} · {ALIGNMENT.get(status.alignment_state, '未知')}\n"
            f"{PHASES.get(status.phase, '未知')} · {position} · 速度源 "
            f"{status.active_velocity_source or '无'}"
        )
        if (not self.active and status.last_result_valid and status.navigation_reason
                and status.last_goal_id != self.last_displayed_result):
            result = RESULT_NAMES.get(status.last_result_code, status.last_result_code)
            self.details.set(f"{result} · {status.navigation_reason}")
            self.last_displayed_result = status.last_goal_id

    def drain_events(self):
        while True:
            try:
                event, value = self.events.get_nowait()
            except Empty:
                break
            if event == "connected":
                self.connected = True
            elif event == "status":
                self.show_status(value)
            elif event == "feedback":
                caption = PHASES.get(value.phase, "未知")
                distance = f" · 剩余 {value.remaining_distance_m:.2f} m" if value.remaining_distance_valid else ""
                self.details.set(f"{caption}{distance} · {value.reason}")
                if value.projected_target_valid:
                    point = value.projected_target.pose.position
                    self.preview_text.set(
                        f"map ({point.x:.3f}, {point.y:.3f}) m · 变换 {value.transform_version}"
                    )
                elif self.mode.get() == "WGS84":
                    self.preview_text.set("投影位置待配准确定")
            elif event == "result":
                self.active = False
                self.details.set(result_text(*value))
            elif event == "grasp_connected":
                self.grasp.connected = True
            elif event == "grasp_status":
                self.grasp.server_status = value
            elif event == "grasp_result":
                self.grasp.finish(*value)
            elif event == "control":
                self.details.set(value)
            elif event == "recording":
                self.record_path = value
                self.record_text.set(f"正在录包 · {value}")
            elif event == "recorded":
                self.recorder = None
                self.record_button.configure(text="开始录包")
                self.record_button.state(["!disabled"])
                caption = "录包已结束" if value in (0, -signal.SIGINT, 130) else f"录包退出码 {value}"
                self.record_text.set(f"{caption} · {self.record_path}")
        if not self.last_status.is_zero() and (rospy.Time.now() - self.last_status).to_sec() > 0.5:
            self.status.set("状态已过期")
        self.update_buttons()
        if not rospy.is_shutdown():
            self.root.after(80, self.drain_events)

    def close(self):
        self.cancel()
        self.grasp.cancel()
        if self.recorder is not None and self.recorder.poll() is None:
            os.killpg(self.recorder.pid, signal.SIGINT)
        self.root.destroy()


if __name__ == "__main__":
    rospy.init_node("slicer_operator_ui")
    window = tk.Tk()
    Operator(window)
    window.mainloop()
