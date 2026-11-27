from pathlib import Path
import os
import sys
import tempfile
import time
import unittest

from PyQt5 import QtTest, QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/slicer_operator_ui/src"))
from slicer_operator_ui.services import ServicePanel


class OperatorServicesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)
        launcher = self.directory / "run"
        launcher.write_text(
            "#!/usr/bin/python3\n"
            "from pathlib import Path\n"
            "import sys\n"
            "directory = Path(__file__).parent\n"
            "command = ' '.join(sys.argv[1:])\n"
            "with (directory / 'commands').open('a') as log:\n"
            "    print(command, file=log)\n"
            "if (directory / 'fail').read_text() == command:\n"
            "    print('模拟服务拒绝')\n"
            "    raise SystemExit(1)\n"
            "if command == 'service status':\n"
            "    print((directory / 'status').read_text())\n",
            encoding="utf-8",
        )
        launcher.chmod(0o700)
        (self.directory / "fail").touch()
        (self.directory / "status").write_text(
            "master active\nbase active\nlocalization active\nnavigation active\nvision inactive"
        )
        self.panel = ServicePanel(str(launcher))
        self.addCleanup(self.panel.deleteLater)

    def click(self, text):
        button = next((item for item in self.panel.findChildren(QtWidgets.QPushButton) if item.text() == text), None)
        self.assertIsNotNone(button, f"Missing workflow button {text}")
        self.assertTrue(button.isEnabled())
        button.click()

    def wait_idle(self):
        deadline = time.monotonic() + 2.0
        while self.panel.operation is not None and time.monotonic() < deadline:
            QtTest.QTest.qWait(10)
        self.assertIsNone(self.panel.operation)

    def commands(self):
        log = self.directory / "commands"
        return [line for line in log.read_text().splitlines() if line != "service status"] if log.exists() else []

    def stop_work(self):
        stop = getattr(self.panel, "stop_work", None)
        self.assertTrue(callable(stop), "Missing workflow stop action")
        stop()
        self.assertTrue(self.panel.stopping_work)

    def test_navigation_button_starts_localization_before_navigation(self):
        self.click("启动导航")
        self.wait_idle()
        self.assertEqual(self.commands(), ["service start localization", "service start navigation"])

    def test_failed_localization_skips_navigation_and_reports_error(self):
        (self.directory / "fail").write_text("service start localization")
        self.click("启动导航")
        self.wait_idle()
        self.assertEqual(self.commands(), ["service start localization"])
        self.assertIn("模拟服务拒绝", self.panel.output.toPlainText())

    def test_stop_button_requests_work_stop_through_operator(self):
        signal = getattr(self.panel, "stop_requested", None)
        self.assertIsNotNone(signal, "Missing operator stop request signal")
        spy = QtTest.QSignalSpy(signal)
        self.click("一键停止作业")
        self.assertEqual(len(spy), 1)
        self.assertEqual(self.commands(), [])

    def test_vision_start_deduplicates_request_queued_during_status_refresh(self):
        self.panel.enter()
        self.assertTrue(self.panel.ensure_vision())
        self.assertTrue(self.panel.ensure_vision())
        self.wait_idle()
        self.assertEqual(self.commands(), ["service start vision"])

    def test_vision_start_failure_notifies_waiting_action(self):
        (self.directory / "fail").write_text("service start vision")
        failure = QtTest.QSignalSpy(self.panel.vision_start_failed)
        self.assertTrue(self.panel.ensure_vision())
        self.wait_idle()
        self.assertEqual(len(failure), 1)
        self.assertIn("模拟服务拒绝", self.panel.output.toPlainText())

    def test_stop_work_replaces_queued_navigation_start(self):
        self.click("启动导航")
        self.stop_work()
        self.wait_idle()
        self.assertEqual(self.commands(), [
            "service start localization", "service stop navigation", "service stop vision", "service stop localization",
        ])
        self.assertFalse(self.panel.stopping_work)

    def test_refused_navigation_stop_skips_stopping_other_services(self):
        (self.directory / "fail").write_text("service stop navigation")
        self.stop_work()
        self.wait_idle()
        self.assertEqual(self.commands(), ["service stop navigation"])
        self.assertIn("模拟服务拒绝", self.panel.output.toPlainText())
        self.assertFalse(self.panel.stopping_work)

    def test_successful_navigation_stop_notifies_operator_during_work_stop(self):
        signal = getattr(self.panel, "navigation_stopped", None)
        self.assertIsNotNone(signal, "Missing navigation stop notification")
        stopping = []
        signal.connect(lambda: stopping.append(self.panel.stopping_work))
        self.stop_work()
        self.wait_idle()
        self.assertEqual(stopping, [True])

    def test_active_navigation_unit_waits_for_action_ready(self):
        self.panel.enter()
        self.wait_idle()
        labels = self.panel.findChildren(QtWidgets.QLabel)
        self.assertTrue(any("等待导航接口" in label.text() for label in labels))
        self.panel.set_navigation_ready(True)
        self.assertTrue(any(label.text() == "已就绪" for label in labels))

    def test_navigation_start_remains_available_if_localization_is_stopped(self):
        (self.directory / "status").write_text(
            "master active\nbase active\nlocalization inactive\nnavigation active\nvision inactive"
        )
        self.panel.enter()
        self.wait_idle()
        self.click("启动导航")
        self.wait_idle()
        self.assertEqual(self.commands(), ["service start localization", "service start navigation"])

    def open_diagnostics(self):
        self.panel.findChild(QtWidgets.QToolButton, "ServiceDiagnostics").click()
        dialog = self.panel.findChild(QtWidgets.QDialog)
        self.assertIsNotNone(dialog, "Diagnostics need a separate window")
        self.assertTrue(dialog.isVisible())
        self.addCleanup(dialog.close)
        return dialog

    def test_diagnostics_open_without_growing_or_blocking_workflow(self):
        main_size = self.panel.sizeHint()
        dialog = self.open_diagnostics()
        self.assertFalse(dialog.isModal())
        self.assertEqual(self.panel.sizeHint(), main_size)

    def test_diagnostics_accesses_master_logs_from_separate_window(self):
        dialog = self.open_diagnostics()
        roles = dialog.findChild(QtWidgets.QComboBox)
        roles.setCurrentIndex(roles.findData("master"))
        logs = next(button for button in dialog.findChildren(QtWidgets.QPushButton) if button.text() == "日志")
        logs.click()
        self.wait_idle()
        self.assertEqual(self.commands(), ["service logs master"])
        self.assertIn("日志完成", dialog.findChild(QtWidgets.QPlainTextEdit).toPlainText())
