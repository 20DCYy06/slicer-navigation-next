"""Touchscreen client for the same service commands used at the shell."""

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
    "not-installed": "未安装",
}


class ServicePanel(QtWidgets.QWidget):
    def __init__(self, executable):
        super().__init__()
        self.executable = executable
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.process_error)
        self.operation = None
        self.running_role = None
        layout = QtWidgets.QVBoxLayout(self)
        self.states = QtWidgets.QLabel("服务入口未配置" if not executable else "等待读取服务")
        self.states.setWordWrap(True)
        layout.addWidget(self.states)
        self.role = QtWidgets.QComboBox()
        for role, name in ROLES.items():
            self.role.addItem(name, role)
        self.role.setCurrentIndex(list(ROLES).index("navigation"))
        layout.addWidget(self.role)
        actions = QtWidgets.QGridLayout()
        self.buttons = []
        for index, (verb, label) in enumerate((
            ("status", "刷新"), ("logs", "日志"), ("start", "启动"), ("stop", "停止"), ("restart", "重启"),
        )):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _checked, action=verb: self.run(action))
            button.setEnabled(bool(executable))
            actions.addWidget(button, index // 3, index % 3)
            self.buttons.append(button)
        layout.addLayout(actions)
        self.output = QtWidgets.QPlainTextEdit(readOnly=True)
        self.output.setPlaceholderText("操作结果与服务日志")
        self.output.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        layout.addWidget(self.output, 1)

    def enter(self):
        self.run("status")

    def run(self, verb):
        if not self.executable or self.operation is not None:
            return
        role = self.role.currentData()
        self.operation = verb
        self.running_role = role
        for button in self.buttons:
            button.setEnabled(False)
        self.role.setEnabled(False)
        args = ["service", verb]
        if verb != "status":
            args.append(role)
            self.output.setPlainText(f"{ROLES[role]} · 执行中")
        self.process.start(self.executable, args)

    def process_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self.output.setPlainText(self.process.errorString())
            self.ready()

    def finished(self, code, _status):
        output = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace").strip()
        if self.operation == "status" and code == 0:
            statuses = {}
            for line in output.splitlines():
                name, _, value = line.partition(" ")
                if name in ROLES:
                    statuses[name] = value
            if statuses:
                self.states.setText("  ·  ".join(
                    f"{ROLES[name]} {STATES.get(value, value)}" for name, value in statuses.items()
                ))
        if self.operation != "status" or code:
            output = output or ("操作完成" if code == 0 else f"退出码 {code}")
            if self.operation == "restart" and self.running_role == "master" and code == 0:
                output += "\nROS Master 已重启，请重新打开界面。"
            self.output.setPlainText(output)
        reload_status = self.operation in ("start", "stop", "restart")
        self.ready()
        if reload_status:
            self.run("status")

    def ready(self):
        self.operation = None
        self.running_role = None
        self.role.setEnabled(True)
        for button in self.buttons:
            button.setEnabled(True)
