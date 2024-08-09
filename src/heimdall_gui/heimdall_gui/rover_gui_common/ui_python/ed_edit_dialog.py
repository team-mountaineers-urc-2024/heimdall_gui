from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from heimdall_gui.urc_gui_common.helpers.validator import Validators, red_if_unacceptable

class ExtremeDeliveryEditWaypointDialog(QDialog):
	def __init__(self, cur_row, cur_name, cur_lat, cur_lon, cur_rad):
		super().__init__()

		self.resize(399, 393)
		self.setWindowTitle("Alter ED Waypoint")

		self.dialog_btns = QDialogButtonBox(self)
		self.dialog_btns.setGeometry(QRect(30, 350, 341, 32))
		self.dialog_btns.setOrientation(Qt.Horizontal)
		self.dialog_btns.setStandardButtons(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
		self.dialog_btns.accepted.connect(self.accept)
		self.dialog_btns.rejected.connect(self.reject)

		### Reorder Waypoint Options #########################################

		self.reorder_btn = QRadioButton("Reorder Waypoint", self)
		self.reorder_btn.setGeometry(QRect(20, 20, 161, 27))
		self.reorder_btn.toggled.connect(self.reorder_toggled)

		self.line = QFrame(self)
		self.line.setGeometry(QRect(20, 40, 351, 20))
		self.line.setFrameShape(QFrame.HLine)
		self.line.setFrameShadow(QFrame.Sunken)

		self.row_label = QLabel("Row:", self)
		self.row_label.setGeometry(QRect(50, 60, 161, 21))
		self.row_entry = QLineEdit(self)
		self.row_entry.setPlaceholderText("Row")
		self.row_entry.setText(str(cur_row))
		self.row_entry.setValidator(Validators.row_validator)
		self.row_entry.textEdited.connect(lambda _: red_if_unacceptable(self.row_entry))
		self.row_entry.setGeometry(QRect(250, 60, 113, 29))
		self.row_entry.setEnabled(False)

		### Edit Waypoint Options ############################################

		self.edit_btn = QRadioButton("Edit Waypoint", self)
		self.edit_btn.setGeometry(QRect(20, 100, 161, 27))
		self.edit_btn.toggled.connect(self.edit_toggled)

		self.line_2 = QFrame(self)
		self.line_2.setGeometry(QRect(20, 120, 351, 20))
		self.line_2.setFrameShape(QFrame.HLine)
		self.line_2.setFrameShadow(QFrame.Sunken)

		self.name_label = QLabel("Name:", self)
		self.name_label.setGeometry(QRect(50, 140, 161, 21))
		self.name_entry = QLineEdit(self)
		self.name_entry.setPlaceholderText("Name")
		self.name_entry.setText(cur_name)
		self.name_entry.textEdited.connect(lambda _: red_if_unacceptable(self.name_entry))
		self.name_entry.setGeometry(QRect(250, 140, 113, 29))
		self.name_entry.setEnabled(False)

		self.lat_label = QLabel("Latitude:", self)
		self.lat_label.setGeometry(QRect(50, 180, 161, 21))
		self.lat_entry = QLineEdit(self)
		self.lat_entry.setPlaceholderText("Latitude")
		self.lat_entry.setText(cur_lat)
		self.lat_entry.setValidator(Validators.latitude_validator)
		self.lat_entry.textEdited.connect(lambda _: red_if_unacceptable(self.lat_entry))
		self.lat_entry.setGeometry(QRect(250, 180, 113, 29))
		self.lat_entry.setEnabled(False)

		self.lon_label = QLabel("Longitude:", self)
		self.lon_label.setGeometry(QRect(50, 220, 161, 21))
		self.lon_entry = QLineEdit(self)
		self.lon_entry.setPlaceholderText("Longitude")
		self.lon_entry.setText(cur_lon)
		self.lon_entry.setValidator(Validators.longitude_validator)
		self.lon_entry.textEdited.connect(lambda _: red_if_unacceptable(self.lon_entry))
		self.lon_entry.setGeometry(QRect(250, 220, 113, 29))
		self.lon_entry.setEnabled(False)

		self.radius_label = QLabel("Radius (m):", self)
		self.radius_label.setGeometry(QRect(50, 260, 161, 21))
		self.radius_entry = QLineEdit(self)
		self.radius_entry.setPlaceholderText("Radius")
		self.radius_entry.setText(cur_rad)
		self.radius_entry.setValidator(Validators.radius_validator)
		self.radius_entry.textEdited.connect(lambda _: red_if_unacceptable(self.radius_entry))
		self.radius_entry.setGeometry(QRect(250, 260, 113, 29))
		self.radius_entry.setEnabled(False)

		### Delete Waypoint Options ##########################################

		self.del_btn = QRadioButton("Delete Waypoint", self)
		self.del_btn.setGeometry(QRect(20, 300, 161, 27))
		self.del_btn.toggled.connect(self.del_toggled)

		self.line_3 = QFrame(self)
		self.line_3.setGeometry(QRect(20, 320, 351, 20))
		self.line_3.setFrameShape(QFrame.HLine)
		self.line_3.setFrameShadow(QFrame.Sunken)

	def reorder_toggled(self):
		if self.reorder_btn.isChecked():
			self.row_entry.setEnabled(True)
		else:
			self.row_entry.setEnabled(False)

	def edit_toggled(self):
		if self.edit_btn.isChecked():
			self.name_entry.setEnabled(True)
			self.lat_entry.setEnabled(True)
			self.lon_entry.setEnabled(True)
			self.radius_entry.setEnabled(True)
		else:
			self.name_entry.setEnabled(False)
			self.lat_entry.setEnabled(False)
			self.lon_entry.setEnabled(False)
			self.radius_entry.setEnabled(False)

	def del_toggled(self):
		pass
