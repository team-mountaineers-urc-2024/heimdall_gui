#!/usr/bin/env python3

from collections import namedtuple
from datetime import datetime as dt
import time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from threading import Thread
from typing import Tuple, List
from enum import IntEnum
import os
import matplotlib.pyplot as graph
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from heimdall_gui.urc_gui_common.helpers.file_helper import file_basenames
from heimdall_gui.urc_gui_common.widgets.graph_widget import Colors

from heimdall_gui.heimdall_ros_link import HeimdallRosLink
from heimdall_gui.ui_python.science_tab import Ui_Science

from heimdall_gui.spectrometer_readings import *

LIGHT_BULB = 0
# TODO figure out how to remove/change these
PRESEALED_ROTATION = -1450
PRESEALED_ROTATION_TOLERANCE = 25
TOP_SAFETY_LIMIT = 60

# ID's being sent to pico subscriber
PUMP_ID = 36
ACTUATOR_ID = 38

# TODO rename these
class Actuators(IntEnum):
	# Not sure what these are
	SITE3_PUMP_TOSCOOP    = 0
	SITE2_PUMP_TOSCOOP    = 1
	SITE1_PUMP_TOSCOOP    = 2
	SITE1_PUMP_TOLAB   	  = 3
	SITE3_PUMP_TOLAB_GPIO = 4 
	SITE2_PUMP_TOLAB      = 5
	PUMP_7    			  = 6		


	FRONT_SCOOP_ACT       = 0
	MIDDLE_SCOOP_ACT      = 1
	BACK_SCOOP_ACT        = 2
	SUBSURFACE_LIN_ACT	  = 3

	# TODO Add a slider
	PROBE_ACT             = 4


class Pumps(IntEnum):
	OFF = 90
	ON = 0

# TODO check this
class Dynamixels(IntEnum):
	SITE1_SCOOP = 1
	SITE2_SCOOP = 2
	SITE3_SCOOP = 3
	SITE4_SCOOP = 4

class LinearActuatorPos(IntEnum):
    TOP = 20
    MIDDLE = 90
    BOTTOM = 180

LIN_ACT_ID_TO_DRUM_ID = {
	Actuators.FRONT_SCOOP_ACT: (Dynamixels.SITE1_SCOOP, Dynamixels.SITE2_SCOOP),
	Actuators.MIDDLE_SCOOP_ACT: (Dynamixels.SITE3_SCOOP, Dynamixels.SITE4_SCOOP),
}

DRUM_ID_TO_LIN_ACT_ID = {
	Dynamixels.SITE1_SCOOP: Actuators.FRONT_SCOOP_ACT,
	Dynamixels.SITE2_SCOOP: Actuators.FRONT_SCOOP_ACT,
	Dynamixels.SITE3_SCOOP: Actuators.MIDDLE_SCOOP_ACT,
	Dynamixels.SITE4_SCOOP: Actuators.MIDDLE_SCOOP_ACT,
}

SpectrometerReading = namedtuple('SpectrometerReading', 'wavelengths intensities line_color')

overlays = {
	'Chlorophyll >500': chlor_over500,
	'Chlorophyll <500': chlor_under500,
	'Egg Shell >500': eggshell_over500,
	'Egg Shell <500': eggshell_under500,
	'Isopropyl Alc >500': isop_over500,
	'Isopropyl Alc <500': isop_under500,
	'Egg Yolk >500': egg_yolk_over500,
	'Egg Yolk <500': egg_yolk_under500,
	'Tangerine >500': tangerine_over500,
	'Tangerine <500': tangerine_under500,
	'Carrot >500': carrot_over500,
	'Carrot <500': carrot_under500,
	'Calcium': calcium_overlay,
	'Potassium': potassium_overlay,
	'Sodium' : sodium_overlay
}

spectrometry_saves_dir = os.path.expanduser('~/.ros/spectrometer_readings')

class ScienceTab(QWidget):
	def __init__(self, roslink: HeimdallRosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.roslink = roslink

		## soil data capturing helper variables ####
		self.moisture_data = {}
		self.temp_data = {}
		self.initial_time = 0

		prev_pico_feedback = ""

		self.roslink.science_log = self.log

		# initialize self.ui
		self.ui = Ui_Science()
		self.ui.setupUi(self)
		self.roslink.gps.connect(self.update_lat_long_alt)
		self.roslink.rover_hdg_degrees.connect(self.update_hdg)
		

		# Set up subsurface motor control
		self.ui.subsurface_motor_on.clicked.connect(lambda: self.subsurface_motor_control(True))
		self.ui.subsurface_motor_off.clicked.connect(lambda: self.subsurface_motor_control(False))

		self.ui.subsurface_motor_off.setChecked(True)

		

		# Setup graph
		self.ui.graph.setup_graph(
			title='Spectrum vs. Wavelength',
			x_label='Wavelength (nm)',
			y_label='Intensity',
			x_range=range(330, 850),
			y_range=range(1000, 12000)
		)

		# Cuvette stuff
		self.ui.move_cuvette_button.clicked.connect(lambda: self.spin_centrifuge(int(self.ui.cuvette_number_choice_spec.currentText())))
		self.ui.move_cuvette_to_burett.clicked.connect(lambda: self.move_cuvette(0))
		self.ui.move_cuvette_to_ipa.clicked.connect(lambda: self.move_cuvette(1))

		# Linear actuator
		self.ui.linear_actuator_slider.sliderReleased.connect(lambda: self.move_actuator())
		# self.ui.middle_linear_actuator_slider.sliderReleased.connect(lambda: self.move_actuator(Actuators.MIDDLE_SCOOP_ACT))
		# self.ui.right_linear_actuator_slider.sliderReleased.connect(lambda: self.move_actuator(Actuators.BACK_SCOOP_ACT))
		# self.ui.subsurface_linear_actuator_slider.sliderReleased.connect(lambda: self.move_actuator(Actuators.SUBSURFACE_LIN_ACT))
		# self.ui.probe_linear_actuator_slider.sliderReleased.connect(lambda: self.move_actuator(Actuators.PROBE_ACT))
		self.linear_actuator_positions = {
			Actuators.FRONT_SCOOP_ACT: 0,
			Actuators.MIDDLE_SCOOP_ACT: 0,
			Actuators.BACK_SCOOP_ACT: 0,
			Actuators.SUBSURFACE_LIN_ACT: 0,
			Actuators.PROBE_ACT: 0
		}
		# self.lin_act_sliders = {
		# 	Actuators.FRONT_SCOOP_ACT: self.ui.left_linear_actuator_slider,
		# 	Actuators.MIDDLE_SCOOP_ACT: self.ui.middle_linear_actuator_slider,
		# 	Actuators.BACK_SCOOP_ACT: self.ui.right_linear_actuator_slider,
		# 	Actuators.SUBSURFACE_LIN_ACT: self.ui.subsurface_linear_actuator_slider,
		# 	Actuators.PROBE_ACT: self.ui.probe_linear_actuator_slider
		# }

		# Pump
		self.ui.pump_button.clicked.connect(self.rotate_pump)

		# Scoop
		self.ui.start_scoop_button.clicked.connect(self.start_scoop)
		self.ui.stop_scoop_button.clicked.connect(self.stop_scoop)
		self.ui.send_scoop_home_button.clicked.connect(self.send_scoop_home)
		self.ui.reboot_button.clicked.connect(self.ui_reboot_scoop)

		#Temp
		self.ui.read_soil_temp.clicked.connect(self.read_temp)

		#Moisture
		self.ui.read_moist_button.clicked.connect(self.read_moisture)

		#Soil data capturing
		self.ui.collect_soil_data.clicked.connect(self.prepare_soil_data_for_graph)

		self.ui.graph_soil_data.clicked.connect(self.graph_soil_data)

		self.ui.clear_soil_data.clicked.connect(self.clear_stored_soil_data)

		# Light switch
		self.ui.light_toggle_checkbox.toggled.connect(self.light_switch)

		# Spectrometer Capture
		self.ui.spectrometer_capture_button.clicked.connect(self.spectrometer_capture)

		# Link cameras to camera funnel
		self.ui.camera1.set_camera_funnel(self.roslink.camera_funnel)
		self.ui.camera2.set_camera_funnel(self.roslink.camera_funnel)
		self.ui.camera3.set_camera_funnel(self.roslink.camera_funnel)

		# Connect overlays
		self.ui.chlorophyll_lt500_overlay.toggled.connect(self.display_chlorophyll_lt500)
		self.ui.chlorophyll_gt500_overlay.toggled.connect(self.display_chlorophyll_gt500)
		self.ui.egg_shell_lt500_overlay.toggled.connect(self.display_egg_shell_lt500)
		self.ui.egg_shell_gt500_overlay.toggled.connect(self.display_egg_shell_gt500)
		self.ui.ipa_lt500_overlay.toggled.connect(self.display_ipa_lt500)
		self.ui.ipa_gt500_overlay.toggled.connect(self.display_ipa_gt500)
		self.ui.egg_yolk_lt500_overlay.toggled.connect(self.display_egg_yolk_lt500)
		self.ui.egg_yolk_gt500_overlay.toggled.connect(self.display_egg_yolk_gt500)
		self.ui.tangerine_lt500_overlay.toggled.connect(self.display_tangerine_lt500)
		self.ui.tangerine_gt500_overlay.toggled.connect(self.display_tangerine_gt500)
		self.ui.carrot_lt500_overlay.toggled.connect(self.display_carrot_lt500)
		self.ui.carrot_gt500_overlay.toggled.connect(self.display_carrot_gt500)
		self.ui.calcium_overlay.toggled.connect(self.display_calcium)
		self.ui.sodium_overlay.toggled.connect(self.display_sodium)
		self.ui.potassium_overlay.toggled.connect(self.display_potassium)

		# Recover history
		self.refresh_history()

		# Connect history list
		self.ui.history_list.currentRowChanged.connect(self.spectrometer_view)
		self.ui.history_list.itemChanged.connect(self.rename_spectrometer_capture)
		self.ui.delete_capture.clicked.connect(self.delete_spectrometer_capture)
		self.ui.clear_history.clicked.connect(self.clear_spectrometer_captures)

		self.ui.subsurface_increment_btn.clicked.connect(self.increment_sub)
		

	### local functions ######################################################

	def log(self, msg):
		self.ui.console_list_widget.addItem(QListWidgetItem(msg))


	### actuators ############################################################

	def move_actuator(self):
		id = 0
		if self.ui.sample_site_selection.currentText() == 'Drill Site':
			id = Actuators.SUBSURFACE_LIN_ACT
		elif self.ui.sample_site_selection.currentText() == 'Probe Site':
			id = Actuators.PROBE_ACT
		elif self.ui.sample_site_selection.currentText() == 'Site 1':
			id = Actuators.FRONT_SCOOP_ACT
		elif self.ui.sample_site_selection.currentText() == 'Site 2':
			id = Actuators.MIDDLE_SCOOP_ACT
		elif self.ui.sample_site_selection.currentText() == 'Site 3':
			id = Actuators.BACK_SCOOP_ACT
		

		def fail_move():
			current_pos = self.ui.linear_actuator_slider.value()
			self.ui.linear_actuator_slider.setSliderPosition(current_pos)

		pos = self.ui.linear_actuator_slider.value()
		

		# Attempt to move actuator
		try:
			feedback = self.roslink.linear_act(ACTUATOR_ID, id, pos)

			if not(feedback is None) and not(feedback == ""):
				self.linear_actuator_positions[id] = pos
				self.log(f"Successfully moved linear actuator to position {pos}")

				# self.log(f"Pico feedback is: {feedback}")

			else:
				self.log("Actuator move failed, pico not available")

				# Move failed, return slider to previous position
				fail_move()

		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Linear Actuator Move Failed",
				f"Failed to move linear actuator to position {pos}",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))
		
			fail_move()
			return
	
	### temp sensor link ###################################################
	def read_temp(self):
		try:
			## Read temp
			feedback = self.roslink.temp_probe()
			if feedback:
				self.ui.temp_data.setProperty("text", f"{feedback}")
				self.log(f"{feedback}")
			else:
				self.log("Try again...")

		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Temp reading failed",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))
		
			return
	
	### soil moisture link #####

	def read_moisture(self):
		try:
			##Read moisture###
			feedback = self.roslink.moisture_sensor()
			if feedback:
				self.ui.moist_data.setProperty("text", f"{feedback}")
				self.log(f"{feedback}")
			else:
				self.log("Try again...")
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Temp reading failed",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))
		
			return
	
	def clear_stored_soil_data(self):
		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Clear soil data?",
			f"Are you sure you want to clear the soil data home?",
			QMessageBox.Yes | QMessageBox.No,
		)
		if (confirmation_box.exec() == QMessageBox.Yes):
			self.moisture_data = {}
			self.temp_data = {}
			self.time_count = 0
			self.log("Currently stored soil data has been cleared")
		
		return


		

	def prepare_soil_data_for_graph(self):
		try:
			cur_moisture = float(self.ui.moist_data.property("text"))
			cur_temp = float(self.ui.temp_data.property("text"))
			if(cur_moisture < 100) or (cur_temp > 50):
				self.log("Moisture level or is not in expected range; please try again...")
				return
			self.log(f"Moisture data ({cur_moisture}) has been captured")
			self.log(f"Temp data {cur_temp} has been captured")


		except:
			self.log("Currently displayed moisture and temp are not numbers; please try again...")
		
		#if these are empty initially
		if(not self.moisture_data.values() and not self.temp_data.values()):
			self.initial_time = time.time()
			self.moisture_data[0] = cur_moisture + self.moisture_data.get(0, 0)
			self.temp_data[0] = cur_temp + self.temp_data.get(0, 0)
		else:
			current_time = time.time()
			self.moisture_data[current_time - self.initial_time] = cur_moisture + self.moisture_data.get(current_time, 0)
			self.temp_data[current_time - self.initial_time] = cur_temp + self.temp_data.get(current_time, 0)

		return

	def graph_soil_data(self):
		def graph_data():
			figure, axis = graph.subplots(1,2)
			axis[0].scatter(self.moisture_data.keys(), self.moisture_data.values())
			axis[0].set_xlabel("Time")
			axis[0].set_ylabel("Moisture Data (Lower Values = More Moist)")
			axis[1].scatter(self.temp_data.keys(), self.temp_data.values())
			axis[1].set_xlabel("Time (S)")
			axis[1].set_ylabel("Temperature (C)")
			graph.show()
		
		thread = Thread(target=graph_data)
		thread.run()

		return


		
		
			
		
			


	

#TODO figure out if this is needed
	# def raise_actuator(self):
	# 	scoop_select = self.ui.site_dropdown.currentIndex()
	# 	if scoop_select < 1:
	# 		QMessageBox(
	# 			QMessageBox.Critical,
	# 			"No scoop selected",
	# 			"Please select a scoop",
	# 			QMessageBox.Ok,
	# 		).exec()
	# 		return


	# 	lin_act_id = DRUM_ID_TO_LIN_ACT_ID[scoop_select]

	# 	try:
	# 		self.roslink.raise_linear_actuator(lin_act_id)
	# 		self.linear_actuator_positions[lin_act_id] = LinearActuatorPos.TOP
	# 		self.lin_act_sliders[lin_act_id].setSliderPosition(LinearActuatorPos.TOP)
	# 		self.log(f"Successfully raised linear actuator")
	# 	except Exception as e:
	# 		QMessageBox(
	# 			QMessageBox.Critical,
	# 			f"Linear Actuator Raise Failed",
	# 			f"Failed to raise linear actuator",
	# 			QMessageBox.Ok,
	# 		).exec()
	# 		self.roslink.get_logger().error(str(e))
	# 		return


# TODO figure out if this is needed too
	# def lower_actuator(self):
	# 	scoop_select = self.ui.site_dropdown.currentIndex()
	# 	if scoop_select < 1:
	# 		QMessageBox(
	# 			QMessageBox.Critical,
	# 			"No scoop selected",
	# 			"Please select a scoop",
	# 			QMessageBox.Ok,
	# 		).exec()
	# 		return

	# 	lin_act_id = DRUM_ID_TO_LIN_ACT_ID[scoop_select]

	# 	try:
	# 		self.roslink.lower_linear_actuator(lin_act_id)
	# 		self.linear_actuator_positions[lin_act_id] = LinearActuatorPos.MIDDLE
	# 		self.lin_act_sliders[lin_act_id].setSliderPosition(LinearActuatorPos.MIDDLE)
	# 		self.log(f"Successfully lowered linear actuator to middle position")
	# 	except Exception as e:
	# 		QMessageBox(
	# 			QMessageBox.Critical,
	# 			f"Linear Actuator Lower Failed",
	# 			f"Failed to lower linear actuator to middle position",
	# 			QMessageBox.Ok,
	# 		).exec()
	# 		self.roslink.get_logger().error(str(e))
	# 		return
		

	def pump(self, pump_num, direction, duration):

		try:
			self.roslink.pump(PUMP_ID, pump_num, direction, duration)


		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Pump Failed",
				f"Failed to set pump to {self.pump_direction}",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))

	def rotate_pump(self):
		self.pump_select = self.ui.pump_choice.currentText()
		self.pump_direction = self.ui.pump_destination_choice.currentText()
		#[0-2]
		dir = -1
		#[0-6]
		id = -1
		
		text = self.pump_select.split()
		if len(text) > 0:
			id = int(text[1]) - 1

		if self.pump_direction == "Stop":
			dir = 0
		if self.pump_direction == "Counterclockwise":
			dir = 1
		if self.pump_direction == "Clockwise":
			dir = 2

		
		try:
			duration_str = self.ui.pump_duration_line_edit.text()
			duration = int(duration_str)
		except ValueError:
			QMessageBox(
				QMessageBox.Critical,
				f"Invalid pump duration",
				f"Please set a valid pump duration (integer)",
				QMessageBox.Ok,
			).exec()
		# 	return

		if id == -1:
			QMessageBox(
				QMessageBox.Critical,
				f"Pump not selected",
				f"Please select a pump",
				QMessageBox.Ok,
			).exec()
			return

		if dir == -1:
			QMessageBox(
				QMessageBox.Critical,
				f"Pump destination not selected",
				f"Please select a pump destination",
				QMessageBox.Ok,
			).exec()
			return
		duration_str = self.ui.pump_duration_line_edit.text()
		duration = int(duration_str)

		self.pump(id, dir, duration)


	def start_scoop(self):
		scoop_select = 0
		
		if self.ui.sample_site_selection.currentText() == 'Site 1':
			scoop_select = 3
		elif self.ui.sample_site_selection.currentText() == 'Site 2':
			scoop_select = 2
		elif self.ui.sample_site_selection.currentText() == 'Site 3':
			scoop_select = 1
		if scoop_select < 1:
			QMessageBox(
				QMessageBox.Critical,
				"No scoop selected or scoop isn't available for selected site",
				"Please select a scoop",
				QMessageBox.Ok,
			).exec()
			return

		# error handling in roslink
		self.roslink.scoop_sample(scoop_select, True)
	
	def stop_scoop(self):
		scoop_select = 0
		
		if self.ui.sample_site_selection.currentText() == 'Site 1':
			scoop_select = 3
		elif self.ui.sample_site_selection.currentText() == 'Site 2':
			scoop_select = 2
		elif self.ui.sample_site_selection.currentText() == 'Site 3':
			scoop_select = 1
		if scoop_select < 1:
			QMessageBox(
				QMessageBox.Critical,
				"No scoop selected",
				"Please select a scoop",
				QMessageBox.Ok,
			).exec()
			return

		# error handling in roslink
		self.roslink.scoop_sample(scoop_select, False)



	## 1 to ipa
	## 0 to burett
	def move_cuvette(self, ipa_or_burette):
		cuvette_id = int(self.ui.cuvette_number_choice_gen.currentText())

		if ipa_or_burette == 1:
			## site 3
			if cuvette_id >= 9 and cuvette_id <= 12:
				cuvette_id -= 2
				cuvette_id = cuvette_id if cuvette_id <= 12 else cuvette_id - 12
				self.spin_centrifuge(cuvette_id=cuvette_id)
			
			## site 1
			elif cuvette_id >= 1 and cuvette_id <= 4:
				cuvette_id += 3
				self.spin_centrifuge(cuvette_id=cuvette_id)
			## site 2
			elif cuvette_id >= 5 and cuvette_id <= 8:
				cuvette_id += 2
				self.spin_centrifuge(cuvette_id=cuvette_id)

			

		elif ipa_or_burette == 0:
			cuvette_id -= 3
			cuvette_id = cuvette_id if cuvette_id > 0 else 12 + cuvette_id
			self.spin_centrifuge(cuvette_id=cuvette_id)
		

		

# TODO figure out if this has changed?
	def spin_centrifuge(self, cuvette_id):
		# Centrigure doesn't work, so the button has been removed and this code
		# has been commented out
		# return

		try:
			success, end = self.roslink.spin_centrifuge(cuvette_id=cuvette_id)
			if success:
				self.log("Successfully spun centrifuge")
				self.log(f"End position is: {end}")
			else:
				raise Exception("Could not spin centrifuge")
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				"Spin Centrifuge Failed",
				"Failed to spin centrifuge",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))

	### drum control #########################################################




	def send_scoop_home(self):
		scoop_select = 0
		
		if self.ui.sample_site_selection.currentText() == 'Site 1':
			scoop_select = 3
		elif self.ui.sample_site_selection.currentText() == 'Site 2':
			scoop_select = 2
		elif self.ui.sample_site_selection.currentText() == 'Site 3':
			scoop_select = 1

		if scoop_select < 1:
			QMessageBox(
				QMessageBox.Critical,
				"No scoop selected",
				"Please select a scoop",
				QMessageBox.Ok,
			).exec()
			return

		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Send scoop home {scoop_select}?",
			f"Are you sure you want to send scoop {scoop_select} home?",
			QMessageBox.Yes | QMessageBox.No,
		)

		if confirmation_box.exec() == QMessageBox.Yes:
			successful = self.roslink.reset_scoop(scoop_select)
			if not(successful):
				self.log(f"Could not reset scoop {scoop_select}")
				QMessageBox(
				QMessageBox.Critical,
				"Failed to reset scoop",
				"Could not reset scoop position to home",
				QMessageBox.Ok,
				).exec()

			else:
				self.log("Successfully reset scoop position to home")

	def ui_reboot_scoop(self):
		scoop_select = 0
		
		if self.ui.sample_site_selection.currentText() == 'Site 1':
			scoop_select = 3
		elif self.ui.sample_site_selection.currentText() == 'Site 2':
			scoop_select = 2
		elif self.ui.sample_site_selection.currentText() == 'Site 3':
			scoop_select = 1

		if scoop_select < 1:
			QMessageBox(
				QMessageBox.Critical,
				"No scoop selected",
				"Please select a scoop",
				QMessageBox.Ok,
			).exec()
			return

		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Reboot drum {scoop_select}?",
			f"Are you sure you want to reboot scoop {scoop_select}?",
			QMessageBox.Yes | QMessageBox.No,
		)

		if confirmation_box.exec() == QMessageBox.Yes:
			successful = self.roslink.reboot_scoop(scoop_select)
			if not(successful):
				self.log(f"Could not reboot scoop {scoop_select}")
				QMessageBox(
				QMessageBox.Critical,
				"Failed to reboot scoop",
				"Could not reboot scoop",
				QMessageBox.Ok,
				).exec()

			else:
				self.log("Successfully reboot scoop")

	### light switch #########################################################
	def light_off(self, timer: QTimer, timer_amount: int):
		try:
			self.roslink.light_switch(0)
			self.ui.light_toggle_checkbox.blockSignals(True)
			self.ui.light_toggle_checkbox.setChecked(False)
			self.ui.light_toggle_checkbox.blockSignals(False)
			self.log(f"Turned off light after {timer_amount / 1000} seconds")
			timer.stop()
			
		except Exception as e:
			self.ui.light_toggle_checkbox.blockSignals(True)
			self.ui.light_toggle_checkbox.setChecked(True)
			self.ui.light_toggle_checkbox.blockSignals(False)
			QMessageBox(
				QMessageBox.Critical,
				f"Light Switch Timer Failed",
				f"Failed to turn off light after {timer_amount / 1000} seconds",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))

	def light_switch(self):
		# 10 seconds
		timer_amount = 10000
		if self.ui.light_toggle_checkbox.isChecked():
			try:
				self.roslink.light_switch(1)
				light_timer = QTimer()
				light_timer.timeout.connect(lambda: self.light_off(light_timer, timer_amount))
				light_timer.start(timer_amount)
			except Exception as e:
				self.ui.light_toggle_checkbox.blockSignals(True)
				self.ui.light_toggle_checkbox.setChecked(False)
				self.ui.light_toggle_checkbox.blockSignals(False)
				QMessageBox(
					QMessageBox.Critical,
					f"Light Switch Failed",
					f"Failed to turn on light",
					QMessageBox.Ok,
				).exec()
				self.roslink.get_logger().error(str(e))
		else:
			try:
				self.roslink.light_switch(0)
			except Exception as e:
				self.ui.light_toggle_checkbox.blockSignals(True)
				self.ui.light_toggle_checkbox.setChecked(True)
				self.ui.light_toggle_checkbox.blockSignals(False)
				QMessageBox(
					QMessageBox.Critical,
					f"Light Switch Failed",
					f"Failed to turn off light",
					QMessageBox.Ok,
				).exec()
				self.roslink.get_logger().error(str(e))

	### overlays #############################################################

	### porphyrins ###

	def display_chlorophyll_lt500(self):
		self.update_overlay(self.ui.chlorophyll_lt500_overlay, 'Chlorophyll <500')

	def display_chlorophyll_gt500(self):
		self.update_overlay(self.ui.chlorophyll_gt500_overlay, 'Chlorophyll >500')

	def display_egg_shell_lt500(self):
		self.update_overlay(self.ui.egg_shell_lt500_overlay, 'Egg Shell <500')

	def display_egg_shell_gt500(self):
		self.update_overlay(self.ui.egg_shell_gt500_overlay, 'Egg Shell >500')

	### controls ###

	def display_ipa_lt500(self):
		self.update_overlay(self.ui.ipa_lt500_overlay, 'Isopropyl Alc <500')

	def display_ipa_gt500(self):
		self.update_overlay(self.ui.ipa_gt500_overlay, 'Isopropyl Alc >500')

	### terpenoids ###

	def display_egg_yolk_lt500(self):
		self.update_overlay(self.ui.egg_yolk_lt500_overlay, 'Egg Yolk <500')

	def display_egg_yolk_gt500(self):
		self.update_overlay(self.ui.egg_yolk_gt500_overlay, 'Egg Yolk >500')

	def display_tangerine_lt500(self):
		self.update_overlay(self.ui.tangerine_lt500_overlay, 'Tangerine <500')

	def display_tangerine_gt500(self):
		self.update_overlay(self.ui.tangerine_gt500_overlay, 'Tangerine >500')

	def display_carrot_lt500(self):
		self.update_overlay(self.ui.carrot_lt500_overlay, 'Carrot <500')

	def display_carrot_gt500(self):
		self.update_overlay(self.ui.carrot_gt500_overlay, 'Carrot >500')
	
	def display_sodium(self):
		self.update_overlay(self.ui.sodium_overlay, 'Sodium')
	
	def display_calcium(self):
		self.update_overlay(self.ui.calcium_overlay, 'Calcium')
	
	def display_potassium(self):
		self.update_overlay(self.ui.potassium_overlay, 'Potassium')

	### base display ###

	def update_overlay(self, overlay_checkbox: QCheckBox, name: str):
		if overlay_checkbox.isChecked():
			reading = overlays[name]
			self.ui.graph.plot(
				reading.wavelengths,
				reading.intensities,
				name,
				reading.line_color
			)
		else:
			self.ui.graph.clear(name)

	### spectrometer #########################################################

	def spectrometer_capture(self):
		try:
			integration_time = int(self.ui.integration_time_edit.text())
		except ValueError:
			self.ui.integration_time_edit.setText("100000")
			integration_time = 100_000

		try:
			spectrometer_response = self.roslink.capture_spectrometer(integration_time)
			if not(spectrometer_response.is_successful):
				raise Exception("Failed to capture with spectrometer")
			
			capture = SpectrometerReading(spectrometer_response.wavelengths, spectrometer_response.intensities, Colors.BLACK)
			self.save_spectrometer_capture(capture)
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Spectrometer Capture Failed",
				f"Failed to capture with spectrometer",
				QMessageBox.Ok,
			).exec()
			self.roslink.get_logger().error(str(e))

	def spectrometer_view(self, row):
		wavelengths, intensities, has_data = self.read_spectrometer_capture(row)
		if has_data:
			self.ui.graph.plot(
				wavelengths,
				intensities,
				'display',
				Colors.BLACK
			)

	def save_spectrometer_capture(self, spectrometer_reading: SpectrometerReading):
		# get filename to save spectrometer info to
		timestamp = dt.now().strftime("%Y-%m-%d_T%H:%M:%S")
		cuvette_name = self.ui.cuvette_number_choice_spec.currentText() or 'Unknown'
		integration_time = self.ui.integration_time_edit.text() or 'Unknown'
		filename = f'{timestamp} - {cuvette_name} - Integration Time: {integration_time}'

		# write to file
		if not os.path.exists(spectrometry_saves_dir):
			os.makedirs(spectrometry_saves_dir)
		filepath = f'{spectrometry_saves_dir}/{filename}.csv'
		data = zip(spectrometer_reading.wavelengths, spectrometer_reading.intensities)
		with open(filepath, 'w') as save_file:
			for wavelength, intensity in data:
				save_file.write(f'{wavelength}, {intensity}\n')

		self.refresh_history()

		 # select most recent entry
		row = self.ui.history_list.count() - 1
		self.ui.history_list.setCurrentRow(row)

		return filepath

	def read_spectrometer_capture(self, row) -> Tuple[List[float], List[float]]:
		# check if a file has been specified
		item = self.ui.history_list.item(row)
		if not item:
			self.ui.graph.clear('display')
			return [], [], False
		filename = item.text()

		# check if file exists
		filepath = f'{spectrometry_saves_dir}/{filename}.csv'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"Capture does not exist",
				f"Capture {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return [], [], False

		# read spectrometry info from file
		wavelengths, intensities = [], []
		with open(filepath, 'r') as save_file:
			for line in save_file:
				wavelength, intensity = map(float, line.split(','))
				wavelengths.append(wavelength)
				intensities.append(intensity)

		return wavelengths, intensities, True
	
	def rename_spectrometer_capture(self, item):
		# check if a file has been specified
		item = self.ui.history_list.currentItem()
		if not item:
			return
		row = self.ui.history_list.currentRow()
		old_filename = self.history[row]

		# check if file exists
		old_filepath = f'{spectrometry_saves_dir}/{old_filename}.csv'
		if not os.path.exists(old_filepath):
			QMessageBox(
				QMessageBox.Critical,
				"Capture does not exist",
				f"Capture {old_filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		new_filename = item.text()
		while new_filename in self.history:
			new_filename = f'{new_filename}-duplicate'
		new_filepath = f'{spectrometry_saves_dir}/{new_filename}.csv'

		os.rename(old_filepath, new_filepath)

		self.refresh_history()

	def delete_spectrometer_capture(self):
		# check if a file has been specified
		item = self.ui.history_list.currentItem()
		if not item:
			QMessageBox(
				QMessageBox.Critical,
				"No capture selected",
				"Set a capture to delete.",
				QMessageBox.Ok,
			).exec()
			return
		row = self.ui.history_list.currentRow()
		filename = item.text()

		# check if file exists
		filepath = f'{spectrometry_saves_dir}/{filename}.csv'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"Capture does not exist",
				f"Capture {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Delete Capture?",
			f"Are you sure you want to delete the spectrometer capture?",
			QMessageBox.Yes | QMessageBox.No,
		)

		# clear capture, with confirmation
		if confirmation_box.exec() == QMessageBox.Yes:
			os.remove(filepath)

			self.refresh_history()

			# select next entry
			self.ui.history_list.setCurrentRow(row)

	def clear_spectrometer_captures(self):
		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Clear Captures?",
			f"Are you sure you want to clear all spectrometer captures?",
			QMessageBox.Yes | QMessageBox.No,
		)

		# clear captures, with confirmation
		if confirmation_box.exec() == QMessageBox.Yes:
			self.ui.graph.clear('display')
			for filename in file_basenames(spectrometry_saves_dir):
				os.remove(f'{spectrometry_saves_dir}/{filename}.csv')

		self.refresh_history()

	def refresh_history(self):
		self.history = file_basenames(spectrometry_saves_dir)
		self.ui.history_list.clear()
		self.ui.history_list.addItems(self.history)
		for i in range(self.ui.history_list.count()):
			item = self.ui.history_list.item(i)
			item.setFlags(item.flags() | Qt.ItemIsEditable)

	# Handles turning on and off the subsurface motor
	def subsurface_motor_control(self, motor_on: bool):
		try: 
			speed = int(self.ui.subsurface_motor_speed.property("text"))
		except:
			self.log("Drill speed input is not an int")
			return

		is_successful = self.roslink.set_sub_motor_state(motor_on, speed)
		state = "on" if motor_on else "off"
		if not(is_successful):
			QMessageBox(
					QMessageBox.Critical,
					f"Subsurface Motor State Change Failed",
					f"Failed to turn the subsurface motor {state}",
					QMessageBox.Ok,
				).exec()
			self.log(f"Failed to turn subsurface motor {state}")

			# If failed trying to turn on
			if motor_on:
				self.ui.subsurface_motor_off.setChecked(True)
				self.ui.subsurface_motor_on.setChecked(False)

			# Failed trying to turn off
			elif not(motor_on):
				self.ui.subsurface_motor_off.setChecked(False)
				self.ui.subsurface_motor_on.setChecked(True)
			return
		else:
			self.log(f"Successfully turned subsurface motor {state}")


		# Uncheck opposite button
		if motor_on:
			self.ui.subsurface_motor_off.setChecked(False)

		else:
			self.ui.subsurface_motor_on.setChecked(False)


	def increment_sub(self):
		inc_amount = self.ui.site_increment_amount.value()
		
		prev_value = self.ui.linear_actuator_slider.value()

		new_value = prev_value + inc_amount

		if new_value <  255:
			self.ui.linear_actuator_slider.setValue(new_value)
			self.move_actuator()

		else:
			self.ui.linear_actuator_slider.setValue(255)
			self.move_actuator()

	def update_lat_long_alt(self, gps: NavSatFix):
		self.ui.alt_data.setProperty("text", f"Altitude: {gps.altitude}m")
		self.ui.lat_data.setProperty("text", f"Latitude: {gps.latitude}")
		self.ui.lng_data.setProperty("text", f"Longitude: {gps.longitude}")
		self.ui.cov_data_east.setProperty("text", f"Position Variance (East): {'{:.2f}'.format(gps.position_covariance[0])}")
		self.ui.cov_data_north.setProperty("text", f"Position Variance (North): {'{:.2f}'.format(gps.position_covariance[4])}")
		self.ui.cov_data_up.setProperty("text", f"Position Variance (Up): {'{:.2f}'.format(gps.position_covariance[8])}")





	
	def update_hdg(self, rover_hdg_degrees: Float64):
		self.ui.compass_hdg.setProperty("text", f"Heading: {rover_hdg_degrees.data} (compass degrees)")
	
	