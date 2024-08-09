from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from rcl_interfaces.msg import Log

from heimdall_gui.urc_gui_common.ui_python.ros_console_widget import Ui_RosConsoleWidget

# http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Log.html
MSG_FMT = "[{name}]<{level}> {msg}"

LEVEL_TO_NAME = {
	10:  "DEBUG",
	20:  "INFO",
	30:  "WARN",
	40:  "ERROR",
	50: "FATAL",
}


class RosConsoleWidget(QWidget):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.ui = Ui_RosConsoleWidget()
		self.ui.setupUi(self)

		self.ui.clear_button.clicked.connect(self.clear_console)

	def log_handler(self, log: Log):
		# self.display_log_message(log.name, log.level, log.level)
		if log.level == 10 and self.ui.debug_checkbox.isChecked() \
		or log.level == 20  and self.ui.info_checkbox.isChecked() \
		or log.level == 30 and self.ui.warn_checkbox.isChecked() \
		or log.level == 40 and self.ui.error_checkbox.isChecked() \
		or log.level == 50 and self.ui.fatal_checkbox.isChecked() \
		:
			self.display_log_message(log.name, log.level, log.msg)

	def display_log_message(self, name, level, msg):
		level_name = LEVEL_TO_NAME[level]
		self.ui.console_list.addItem(QListWidgetItem(MSG_FMT.format(name=name, level=level_name, msg=msg)))

	def clear_console(self):
		self.ui.console_list.clear()
