#!/usr/bin/env python3

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from heimdall_gui.heimdall_ros_link import HeimdallRosLink

from heimdall_gui.ui_python.heimdall_controls_tab import Ui_HeimdallControls


class HeimdallControlsTab(QWidget):
	def __init__(self, roslink: HeimdallRosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.roslink = roslink

		# initialize ui
		self.ui = Ui_HeimdallControls()
		self.ui.setupUi(self)

		# update triggers
		self.ui.drive_backward_checkbox.clicked.connect(self.set_drive_direction)
		self.ui.car_checkbox.clicked.connect(self.set_turning_style)
		self.ui.velocity_control_checkbox.clicked.connect(self.set_arm_control)
		self.ui.arm_safety_checkbox.toggled.connect(self.set_arm_safety_features)
		self.ui.arm_speed_multipler_slider.sliderReleased.connect(self.set_arm_speed_multiplier)
		self.ui.use_obstacle_avoidance_checkbox.toggled.connect(self.set_obav_usage)
		self.ui.pid_enable_button.clicked.connect(self.enable_pid_planner)
		self.ui.pid_disable_button.clicked.connect(self.disable_pid_planner)
		self.ui.dwa_enable_button.clicked.connect(self.enable_dwa_planner)
		self.ui.dwa_disable_button.clicked.connect(self.disable_dwa_planner)

		# memory
		self.last_speed_multiplier = .5

		# compass
		self.roslink.current_goal.connect(self.ui.compass.goal_handler)
		self.roslink.gps.connect(self.ui.compass.gps_handler)
		self.roslink.pose.connect(self.ui.compass.pose_handler)

	### slots ################################################################

	def set_drive_direction(self):
		forward = self.ui.drive_backward_checkbox.isChecked()
		try:
			self.roslink.change_drive_direction(forward)
			QMessageBox(
				QMessageBox.Information,
				f"Drive Direction Change Succeeded",
				f"Successfuly updated drive direction.",
				QMessageBox.Ok,
			).exec()

		# TODO Figure out what this exception would be in ROS2
			#rospy.service.ServiceException
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Drive Direction Change Failed",
				f"Failed to update drive direction.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

			# undo checkbox change
			self.ui.drive_backward_checkbox.blockSignals(True)
			self.ui.drive_backward_checkbox.toggle()
			self.ui.drive_backward_checkbox.blockSignals(False)

	def set_turning_style(self):
		car = self.ui.car_checkbox.isChecked()
		try:
			self.roslink.change_car_turning_style(car)
			QMessageBox(
				QMessageBox.Information,
				f"Turning Style Change Succeeded",
				f"Successfully updated turning style.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Turning Style Change Failed",
				f"Failed to update turning style.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

			# undo checkbox change
			self.ui.car_checkbox.blockSignals(True)
			self.ui.car_checkbox.toggle()
			self.ui.car_checkbox.blockSignals(False)

	def set_arm_control(self):
		velocity = self.ui.velocity_control_checkbox.isChecked()
		try:
			self.roslink.change_arm_control(velocity)
			QMessageBox(
				QMessageBox.Information,
				f"Arm Control Change Succeeded",
				f"Successfully updated arm control type.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Arm Control Change Failed",
				f"Failed to update arm control type.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

			# undo checkbox change
			self.ui.velocity_control_checkbox.blockSignals(True)
			self.ui.velocity_control_checkbox.toggle()
			self.ui.velocity_control_checkbox.blockSignals(False)

	def set_arm_safety_features(self):
		enable_arm_safety = self.ui.arm_safety_checkbox.isChecked()
		try:
			self.roslink.change_arm_safety_features(enable_arm_safety)
			QMessageBox(
				QMessageBox.Information,
				f"Arm Safety Change Succeeded",
				f"Successfully updated arm safety features.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Arm Safety Change Failed",
				f"Failed to update arm safety features.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

			# undo checkbox change
			self.ui.arm_safety_checkbox.blockSignals(True)
			self.ui.arm_safety_checkbox.toggle()
			self.ui.arm_safety_checkbox.blockSignals(False)

	def set_arm_speed_multiplier(self):
		multipler = self.ui.arm_speed_multipler_slider.value() / 10  # slider doesn't support float
		try:
			self.roslink.change_arm_speed_multiplier(multipler)
			QMessageBox(
				QMessageBox.Information,
				f"Arm Speed Multipler Change Succeeded",
				f"Successfully updated arm speed.",
				QMessageBox.Ok,
			).exec()
			self.last_speed_multiplier = multipler
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Arm Speed Change Failed",
				f"Failed to update arm speed.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

			# undo slider change
			self.ui.arm_speed_multipler_slider.blockSignals(True)
			self.ui.arm_speed_multipler_slider.setSliderPosition(self.last_speed_multiplier * 10)
			self.ui.arm_speed_multipler_slider.blockSignals(False)

	def set_obav_usage(self):
		use_obav = self.ui.use_obstacle_avoidance_checkbox.isChecked()
		planner = self.roslink.dwa_planner_name if use_obav else self.roslink.pid_planner_name
		try:
			# self.roslink.set_planner(planner)
			QMessageBox(
				QMessageBox.Information,
				"Planner Change Succeeded",
				"Successfully changed planners.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				"Planner Change Failed",
				"Failed to change planners.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

			# undo checkbox change
			self.ui.use_obstacle_avoidance_checkbox.blockSignals(True)
			self.ui.use_obstacle_avoidance_checkbox.toggle()
			self.ui.use_obstacle_avoidance_checkbox.blockSignals(False)

	def enable_pid_planner(self):
		try:
			self.roslink.enable_pid_planner(True)
			QMessageBox(
				QMessageBox.Information,
				"PID Planner Enable Succeeded",
				"Successfully enabled PID Planner.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				"PID Planner Enable Failed",
				"Failed to enable PID Planner.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

	def disable_pid_planner(self):
		try:
			self.roslink.enable_pid_planner(False)
			QMessageBox(
				QMessageBox.Information,
				"PID Planner Disable Succeeded",
				"Successfully disabled PID Planner.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				"PID Planner Disable Failed",
				"Failed to disable PID Planner.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

	def enable_dwa_planner(self):
		try:
			self.roslink.enable_dwa_planner(True)
			QMessageBox(
				QMessageBox.Information,
				"DWA Planner Enable Succeeded",
				"Successfully enabled DWA Planner.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				"DWA Planner Enable Failed",
				"Failed to enable DWA Planner.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)

	def disable_dwa_planner(self):
		try:
			self.roslink.enable_dwa_planner(False)
			QMessageBox(
				QMessageBox.Information,
				"DWA Planner Disable Succeeded",
				"Successfully disabled DWA Planner.",
				QMessageBox.Ok,
			).exec()
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				"DWA Planner Disable Failed",
				"Failed to disable DWA Planner.",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger.error(e)
