"""Touchscreen client for the same service commands used at the shell."""

from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets


ROLES = {
    "master": "ROS Master",
    "base": "底盘",
    "localization": "定位",
    "navigation": "导航",
    "vision": "视觉",
}
STATES = {
    "active": "运行",
    "inactive": "停止",
    "failed": "失败",
    "activating": "启动中",
    "deactivating": "停止中",
    "reloading": "加载中",
    "not-installed": "未安装",
    "unavailable": "状态不可用",
}
VERBS = {"status": "刷新", "logs": "日志", "start": "启动", "stop": "停止", "restart": "重启"}
MUTATIONS = ("start", "stop", "restart")
STARTED_STATES = ("active", "activating")


class ServicePanel(QtWidgets.QWidget):
    stop_requested = QtCore.pyqtSignal()
    navigation_stopped = QtCore.pyqtSignal()
    vision_start_failed = QtCore.pyqtSignal(str)

    def __init__(self, executable):
        super().__init__(objectName="ServicePanel")
        self.executable = executable
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.process_error)
        self.operation = None
        self.running_role = None
        self.stopping_work = False
        self._stop_step = False
        self._pending = deque()
        self._states = {}
        self._navigation_ready = False
        self._status_timer = QtCore.QTimer(self, interval=10000)
        self._status_timer.timeout.connect(self.enter)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        workflow = QtWidgets.QGridLayout()
        workflow.setHorizontalSpacing(6)
        workflow.setVerticalSpacing(3)
        workflow.setColumnStretch(0, 1)
        self._launch_buttons = {}
        self._state_labels = {}
        for row, (role, label, detail) in enumerate((
            ("localization", "启动定位", "启动雷达、LIO 和感知。室外配置同时启动 RTK，配准由 WGS84 任务执行。"),
            ("base", "启动底盘", "启动底盘服务。"),
            ("navigation", "启动导航", "先启动定位，再启动导航。"),
            ("vision", "启动视觉", "启动视觉服务，然后在视觉操作区选择抓取或释放。"),
        )):
            button = QtWidgets.QPushButton(label, objectName="WorkflowAction")
            button.setToolTip(detail)
            button.clicked.connect(lambda _checked, name=role: self.start_work(name))
            self._launch_buttons[role] = button
            state = QtWidgets.QLabel(objectName="WorkflowState")
            self._state_labels[role] = state
            workflow.addWidget(button, row, 0)
            workflow.addWidget(state, row, 1)
        aircraft = QtWidgets.QLabel("飞机通信 V1 / V2", objectName="WorkflowAircraft")
        aircraft.setToolTip("当前版本未接入飞机通信。")
        workflow.addWidget(aircraft, 4, 0)
        workflow.addWidget(QtWidgets.QLabel("未接入", objectName="WorkflowState"), 4, 1)
        self.stop_button = QtWidgets.QPushButton("一键停止作业", objectName="WorkflowStop")
        self.stop_button.setToolTip("取消作业，确认车辆停稳后停止导航、视觉和定位。")
        self.stop_button.clicked.connect(self.stop_requested.emit)
        workflow.addWidget(self.stop_button, 5, 0, 1, 2)
        layout.addLayout(workflow)

        diagnostics = QtWidgets.QToolButton(text="服务诊断", objectName="ServiceDiagnostics")
        diagnostics.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        diagnostics.setArrowType(QtCore.Qt.RightArrow)
        layout.addWidget(diagnostics)
        details = QtWidgets.QDialog(self, windowTitle="服务诊断")
        details.resize(440, 300)
        diagnostics.clicked.connect(details.show)
        actions = QtWidgets.QGridLayout(details)
        actions.setContentsMargins(10, 10, 10, 10)
        actions.setSpacing(4)
        actions.setRowStretch(4, 1)
        self.states = QtWidgets.QLabel()
        self.states.setWordWrap(True)
        actions.addWidget(self.states, 0, 0, 1, 3)
        self.role = QtWidgets.QComboBox()
        for role, label in ROLES.items():
            self.role.addItem(label, role)
        self.role.setCurrentIndex(list(ROLES).index("navigation"))
        actions.addWidget(self.role, 1, 0, 1, 3)
        self.buttons = []
        for index, (verb, label) in enumerate(VERBS.items()):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _checked, action=verb: self.run(action))
            actions.addWidget(button, 2 + index // 3, index % 3)
            self.buttons.append(button)

        self.output = QtWidgets.QPlainTextEdit(readOnly=True, objectName="WorkflowLog")
        self.output.setPlaceholderText("操作结果与服务日志")
        self.output.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.output.setMaximumBlockCount(120)
        self.output.setFixedHeight(58)
        self.output.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        layout.addWidget(self.output)
        detail_log = QtWidgets.QPlainTextEdit(readOnly=True)
        detail_log.setDocument(self.output.document())
        detail_log.setFont(self.output.font())
        detail_log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        detail_log.setPlaceholderText(self.output.placeholderText())
        actions.addWidget(detail_log, 4, 0, 1, 3)
        close = QtWidgets.QPushButton("关闭")
        close.clicked.connect(details.close)
        actions.addWidget(close, 5, 2)
        self._display()

    def showEvent(self, event):
        super().showEvent(event)
        self.enter()

    def enter(self):
        if self.executable and not self._status_timer.isActive():
            self._status_timer.start()
        self.run("status")

    def set_navigation_ready(self, ready):
        if ready != self._navigation_ready:
            self._navigation_ready = ready
            self._display()

    def start_work(self, role):
        roles = ("localization", "navigation") if role == "navigation" else (role,)
        self._begin("start", roles)

    def ensure_vision(self):
        if not self.executable or self.stopping_work:
            return False
        command = ("start", "vision")
        if (self.operation, self.running_role) == command or command in self._pending:
            return True
        if self.operation == "status":
            self._pending.append(command)
        elif self.operation is None:
            self.start_work("vision")
        else:
            return False
        return True

    def stop_work(self):
        if not self.executable or self.stopping_work:
            return
        self.stopping_work = True
        self._pending.clear()
        self._pending.extend(("stop", role) for role in ("navigation", "vision", "localization"))
        self.output.clear()
        if self.operation is None:
            self._next_command()
        else:
            self._display()

    def run(self, verb):
        role = None if verb == "status" else self.role.currentData()
        self._begin(verb, (role,))

    def _begin(self, verb, roles):
        if not self.executable or self.operation is not None:
            return
        if verb != "status":
            self.output.clear()
        self._pending.extend((verb, role) for role in roles)
        self._next_command()

    def _next_command(self):
        self.operation, self.running_role = self._pending.popleft()
        self._stop_step = self.stopping_work
        args = ["service", self.operation]
        if self.running_role is not None:
            args.append(self.running_role)
            self.output.appendPlainText(f"{VERBS[self.operation]}{ROLES[self.running_role]}中…")
        self._display()
        self.process.start(self.executable, args)

    def process_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self._complete(False, self.process.errorString(), refresh=False)

    def finished(self, code, status):
        output = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace").strip()
        success = status == QtCore.QProcess.NormalExit and code == 0
        self._complete(success, output or (f"退出码 {code}" if not success else ""))

    def _complete(self, success, output, refresh=True):
        verb, role = self.operation, self.running_role
        if verb == "status" and success:
            self._states = {}
            notices = []
            for line in output.splitlines():
                name, _, state = line.partition(" ")
                if name in ROLES:
                    self._states[name] = state
                else:
                    notices.append(line)
            self.states.setToolTip("\n".join(notices))
        elif verb != "status":
            result = "完成" if success else "失败"
            self.output.appendPlainText(f"{ROLES[role]} · {VERBS[verb]}{result}")
            if output:
                self.output.appendPlainText(output)
            if success and verb == "restart" and role == "master":
                self.output.appendPlainText("ROS Master 已重启，请重新打开界面。")
            if success and verb == "stop" and role in ("navigation", "base", "master"):
                self.navigation_stopped.emit()
            if not success and verb == "start" and role == "vision":
                self.vision_start_failed.emit("视觉服务启动失败，请查看服务日志")
        else:
            self._states = dict.fromkeys(ROLES, "unavailable")
            self.output.appendPlainText(f"读取服务失败 · {output}")

        # A stop queued during another command still runs if that earlier command fails.
        if not success and verb != "status" and (not self.stopping_work or self._stop_step):
            self._pending.clear()
        self.operation = self.running_role = None
        if self._pending:
            self._next_command()
            return
        self.stopping_work = False
        self._display()
        if refresh and verb in MUTATIONS:
            self.enter()

    def _display(self):
        idle = bool(self.executable) and self.operation is None
        default = "等待读取" if self.executable else "未配置"
        for role, label in self._state_labels.items():
            state = self._states.get(role)
            description = STATES.get(state, default)
            if self.running_role == role and self.operation in MUTATIONS:
                description, state = f"{VERBS[self.operation]}中", "activating"
            elif role == "navigation" and state == "active":
                description = "已就绪" if self._navigation_ready else "运行 · 等待导航接口"
                state = state if self._navigation_ready else "activating"
            label.setText(description)
            if label.property("state") != state:
                label.setProperty("state", state)
                label.style().unpolish(label)
                label.style().polish(label)
            started = self._states.get(role) in STARTED_STATES
            if role == "navigation":
                started = started and self._states.get("localization") in STARTED_STATES
            self._launch_buttons[role].setEnabled(idle and not started)
        self.stop_button.setEnabled(bool(self.executable) and not self.stopping_work)
        self.role.setEnabled(idle)
        for button in self.buttons:
            button.setEnabled(idle)
        self.states.setText("  ·  ".join(
            f"{name} {STATES.get(self._states.get(role), default)}" for role, name in ROLES.items()
        ))
