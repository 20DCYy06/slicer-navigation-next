"""Embedded top-down RViz with a local preview tool."""

from pathlib import Path

from PyQt5 import QtCore, QtWidgets
import rospkg


class MapPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.layout_box = QtWidgets.QVBoxLayout(self)
        self.layout_box.setContentsMargins(0, 0, 0, 0)
        toolbar = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("实时地图")
        title.setObjectName("SectionTitle")
        toolbar.addWidget(title)
        toolbar.addStretch()
        self.move = QtWidgets.QPushButton("移动视图", checkable=True)
        self.select = QtWidgets.QPushButton("地图选点", checkable=True)
        self.tool_buttons = QtWidgets.QButtonGroup(self)
        for index, button in enumerate((self.move, self.select)):
            self.tool_buttons.addButton(button, index)
            toolbar.addWidget(button)
            button.setEnabled(False)
        self.move.setChecked(True)
        self.layout_box.addLayout(toolbar)

        self.canvas = QtWidgets.QLabel("等待 ROS Master")
        self.canvas.setAlignment(QtCore.Qt.AlignCenter)
        self.layout_box.addWidget(self.canvas, 1)
        self.frame = None

    def connect_rviz(self, click_topic, preview_topic):
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
        self.tools.getTool(1).getPropertyContainer().subProp("Topic").setValue(click_topic)
        self.marker = manager.getRootDisplayGroup().getDisplayAt(4)
        self.marker.subProp("Topic").setValue(preview_topic)
        self.marker.setEnabled(False)
        self.move.clicked.connect(lambda: self.tools.setCurrentTool(self.tools.getTool(0)))
        self.select.clicked.connect(lambda: self.tools.setCurrentTool(self.tools.getTool(1)))
        self.tools.setDefaultTool(self.tools.getTool(0))
        self.tools.setCurrentTool(self.tools.getTool(0))
        self.move.setEnabled(True)
        self.select.setEnabled(True)
        self.layout_box.replaceWidget(self.canvas, frame)
        self.canvas.deleteLater()
        self.frame = frame

    def select_move(self):
        if self.frame is not None:
            self.move.click()

    def show_marker(self, visible):
        if self.frame is not None:
            self.marker.setEnabled(visible)

    def close_map(self):
        if self.frame is not None:
            self.frame.getManager().stopUpdate()
