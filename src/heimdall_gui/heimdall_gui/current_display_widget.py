from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from std_msgs.msg import Float32MultiArray

from .ui_python.current_display import Ui_CurrentDisplay

from heimdall_gui.heimdall_ros_link import HeimdallRosLink

class CurrentDisplayWidget(QWidget):
	def __init__(self, roslink: HeimdallRosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.ui = Ui_CurrentDisplay()
		self.ui.setupUi(self)

		self.roslink = roslink
		self.roslink.drivetrain_currents.connect(self.display_drivetrain_currents)
		self.roslink.arm_currents.connect(self.display_arm_currents)

		### set labels #######################################################

		self.ui.gr.ui.label.setText("GR")
		self.ui.rl.ui.label.setText("RL")
		self.ui.dl.ui.label.setText("DL")
		self.ui.dr.ui.label.setText("DR")
		self.ui.sh.ui.label.setText("SH")
		self.ui.el.ui.label.setText("EL")

		self.ui.lf.ui.label.setText("LF")
		self.ui.lb.ui.label.setText("LB")
		self.ui.rf.ui.label.setText("RF")
		self.ui.rb.ui.label.setText("RB")

		### set ranges #######################################################

		self.ui.gr.set_current_range(min_f=0, max_f=2, precision=2)
		self.ui.rl.set_current_range(min_f=0, max_f=2, precision=2)
		self.ui.dl.set_current_range(min_f=0, max_f=2, precision=2)
		self.ui.dr.set_current_range(min_f=0, max_f=2, precision=2)
		self.ui.sh.set_current_range(min_f=0, max_f=5, precision=2)
		self.ui.el.set_current_range(min_f=0, max_f=5, precision=2)

		self.ui.lf.set_current_range(min_f=0, max_f=5, precision=2)
		self.ui.lb.set_current_range(min_f=0, max_f=5, precision=2)
		self.ui.rf.set_current_range(min_f=0, max_f=5, precision=2)
		self.ui.rb.set_current_range(min_f=0, max_f=5, precision=2)

		### set initial displays #############################################

		self.ui.gr.set_current(0.50)
		self.ui.rl.set_current(0.50)
		self.ui.dl.set_current(0.50)
		self.ui.dr.set_current(0.50)
		self.ui.sh.set_current(0.50)
		self.ui.el.set_current(0.50)

		self.ui.lf.set_current(0.50)
		self.ui.lb.set_current(0.50)
		self.ui.rf.set_current(0.50)
		self.ui.rb.set_current(0.50)

	def display_drivetrain_currents(self, currents: Float32MultiArray):
		data = currents.data
		self.ui.lf.set_current(abs(data[0]))
		self.ui.lb.set_current(abs(data[1]))
		self.ui.rf.set_current(abs(data[2]))
		self.ui.rb.set_current(abs(data[3]))

	def display_arm_currents(self, currents: Float32MultiArray):
		data = currents.data
		self.ui.sh.set_current(abs(data[0]))
		self.ui.el.set_current(abs(data[1]))
