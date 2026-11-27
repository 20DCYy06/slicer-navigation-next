"""Embedded top-down RViz with independent navigation and waypoint tools."""

from pathlib import Path

from PyQt5 import QtCore, QtWidgets
import rospkg


TOOLBAR_STYLE = """
QToolBar { background: #172534; padding: 5px; border-bottom: 1px solid #344d60; spacing: 7px; }
QToolButton { color: #e9f3fc; padding: 7px 12px; background: #243a4b; border-radius: 4px; }
QToolButton:checked { background: #79cfed; color: #0a2030; }
QToolButton:disabled { color: #8499a4; }
"""


class MapPanel(QtWidgets.QWidget):
    close_requested = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.layout_box = QtWidgets.QVBoxLayout(self)
        self.layout_box.setContentsMargins(0, 0, 0, 0)
        self.layout_box.setSpacing(0)
        toolbar = QtWidgets.QToolBar("导航交互", self)
        toolbar.setMovable(False)
        toolbar.setStyleSheet(TOOLBAR_STYLE)
        tool_group = QtWidgets.QActionGroup(toolbar)
        tool_group.setExclusive(True)
        self.tool_actions = []
        for index, text in enumerate(("移动视图", "2D Nav Goal", "Waypoint 打点")):
            action = toolbar.addAction(text)
            action.setCheckable(True)
            action.setEnabled(False)
            action.triggered.connect(lambda _checked, tool_index=index: self.select_tool(tool_index))
            tool_group.addAction(action)
            self.tool_actions.append(action)
        self.tool_actions[0].setChecked(True)
        toolbar.addSeparator()
        toolbar.addAction("关闭", lambda: self.close_requested.emit())
        self.layout_box.addWidget(toolbar)

        self.canvas = QtWidgets.QLabel("等待 ROS Master")
        self.canvas.setAlignment(QtCore.Qt.AlignCenter)
        self.layout_box.addWidget(self.canvas, 1)
        self.frame = None

    def connect_rviz(self, click_topic, preview_topic, waypoint_topic):
        # Same embedded VisualizationFrame as the earlier RTK desktop.
        from rviz import bindings as rviz

        config_path = Path(rospkg.RosPack().get_path("slicer_operator_ui")) / "config/operator.rviz"
        config = rviz.Config()
        reader = rviz.YamlConfigReader()
        reader.readFile(config, str(config_path))
        if reader.error():
            raise RuntimeError(reader.errorMessage())

        frame = rviz.VisualizationFrame()
        frame.setSplashPath("")
        frame.initialize()
        frame.load(config)
        frame.setHideButtonVisibility(False)
        frame.menuBar().hide()
        frame.statusBar().hide()
        for dock in frame.findChildren(QtWidgets.QDockWidget):
            dock.hide()
        for toolbar in frame.findChildren(QtWidgets.QToolBar):
            toolbar.hide()
        frame.setMinimumSize(0, 0)
        manager = frame.getManager()
        self.tools = manager.getToolManager()
        self.rviz_tools = tuple(self.tools.getTool(index) for index in range(len(self.tool_actions)))
        for tool, topic in zip(self.rviz_tools[1:], (click_topic, waypoint_topic)):
            tool.getPropertyContainer().subProp("Topic").setValue(topic)
        self.marker = manager.getRootDisplayGroup().getDisplayAt(4)
        self.marker.subProp("Topic").setValue(preview_topic)
        self.marker.setEnabled(False)
        self.layout_box.replaceWidget(self.canvas, frame)
        self.canvas.deleteLater()
        self.frame = frame
        frame.show()
        self.select_move()
        # Noetic's Python bindings cannot connect to the C++ toolChanged signal.
        self.tool_timer = QtCore.QTimer(self, interval=100)
        self.tool_timer.timeout.connect(self.mark_current_tool)
        self.tool_timer.start()
        for action in self.tool_actions:
            action.setEnabled(True)

    def select_tool(self, index):
        move, _, waypoint = self.rviz_tools
        tool = self.rviz_tools[index]
        self.tools.setDefaultTool(waypoint if tool == waypoint else move)
        self.tools.setCurrentTool(tool)
        self.mark_current_tool()

    def mark_current_tool(self):
        current = self.tools.getCurrentTool()
        for tool, action in zip(self.rviz_tools, self.tool_actions):
            if tool == current:
                action.setChecked(True)
                break

    def select_move(self):
        if self.frame is not None:
            self.select_tool(0)

    def show_marker(self, visible):
        if self.frame is not None:
            self.marker.setEnabled(visible)

    def close_map(self):
        if self.frame is not None:
            self.tool_timer.stop()
            self.frame.getManager().stopUpdate()
