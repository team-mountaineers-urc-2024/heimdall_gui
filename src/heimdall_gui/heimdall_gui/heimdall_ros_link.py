#!/usr/bin/env python3

from heimdall_gui.rover_gui_common import RoverRosLink

import rclpy
import rclpy.qos
from rclpy.node import Node, Publisher

from std_msgs.msg import Float32MultiArray, String, UInt8MultiArray, Float64
from std_srvs.srv import SetBool

from robot_interfaces.srv import SpinCentrifuge, CollectSample, CollectSpectrometerData, SubsurfaceMotor, SendScoopHome, RebootScoop

from PyQt5.QtCore import pyqtSignal as Signal

import time

from threading import Thread

from PyQt5.QtWidgets import *


# thread decorator, moves function into thread
def threaded_decorator(func):
	def inner(*args, **kwargs):
         
    	# Move the passed function into a thread and run it
		thread = Thread(target=lambda: func(*args, **kwargs))
		thread.run()
		 	
	return inner


class HeimdallRosLink(RoverRosLink):
	"""Supply signals for the Qt app, originating from topic callbacks."""

	arm_currents = Signal(Float32MultiArray)
	drivetrain_currents = Signal(Float32MultiArray)

	pid_planner_status = Signal(String)
	dwa_planner_status = Signal(String)

	science_log = None

	def __init__(self, executor):
		super().__init__(executor)

		self.pico_feedback = ""

		### autonomy #########################################################

		self.pid_planner_name = self.ROSnode.get_parameter("pid_planner_name").value
		self.dwa_planner_name = self.ROSnode.get_parameter("dwa_planner_name").value

		# pid_planner_status_topic = self.ROSnode.get_parameter("pid_planner_status_topic").value
		# dwa_planner_status_topic = self.ROSnode.get_parameter("dwa_planner_status_topic").value

		# select_planner_service = self.ROSnode.get_parameter("select_planner_service").value
		pid_planner_enabled_service = self.ROSnode.get_parameter("pid_planner_enabled_service").value
		dwa_planner_enabled_service = self.ROSnode.get_parameter("dwa_planner_enabled_service").value

		# self.pid_planner_status_sub = self.make_subscriber(pid_planner_status_topic, String, self.pid_planner_status)
		# self.dwa_planner_status_sub = self.make_subscriber(dwa_planner_status_topic, String, self.dwa_planner_status)

		# self.select_planner_serv = self.make_service(select_planner_service, SetPlanner, "select_planner_service_client")
		self.enable_pid_planner_serv = self.make_service(pid_planner_enabled_service, SetBool, "pid_planner_enabled_service_client")
		self.enable_dwa_planner_serv = self.make_service(dwa_planner_enabled_service, SetBool, "dwa_planner_enabled_service_client")

		### drivetrain #######################################################

		drivetrain_currents_topic = "drivetrain_current_data"

		self.drivetrain_currents_sub = self.make_subscriber(drivetrain_currents_topic, Float32MultiArray, self.drivetrain_currents)

		### arm ##############################################################

		arm_currents_topic = "arm_current"
		arm_control_service = "UseVeloControl"
		# arm_speed_multipler_service = "SetSpeedMultiplier"
		arm_safety_check_service = "UseArmSafetyCheck"

		self.arms_currents_sub = self.make_subscriber(arm_currents_topic, Float32MultiArray, self.arm_currents)
		self.arm_control_serv = self.make_service(arm_control_service, SetBool)
		# self.arm_speed_multiplier_serv = self.make_service(arm_speed_multipler_service, SetFloat)
		self.arm_safety_check_serv = self.make_service(arm_safety_check_service, SetBool)

		### science ##########################################################
		self.pico_sub = self.make_subscriber('pico_feedback', String, self.set_curr_feedback, False)

		# self.linear_act_serv = self.make_service("move_linear_actuator", MoveLinearActuator)
		self.pico_pub:Publisher = self.ROSnode.create_publisher(UInt8MultiArray, 'pico_sub',10)
		# self.pump_pub:Publisher = self.ROSnode.create_publisher(MovePump, "pico_sub", qos_profile=10)

		self.scoop_sample_serv = self.make_service("collect_sample", CollectSample)
		self.reboot_scoop_serv = self.make_service("reboot_scoop", RebootScoop)

		# self.move_cuvette_serv = self.make_service("move_cuvette", MoveCuvette)
		self.spin_centrifuge_serv = self.make_service("spin_centrifuge", SpinCentrifuge)

		self.spectrometer_serv = self.make_service("spectrometer", CollectSpectrometerData)

		#TODO change reboot drum to send collector home service
		self.send_scoop_home = self.make_service("send_collector_home", SendScoopHome)

		self.subsurface_motor_serv = self.make_service("SubsurfaceMotor", SubsurfaceMotor)

	### autonomy #############################################################

	# def set_planner(self, planner: str) -> SetPlanner.Response:
	# 	return self.select_planner_serv.send_request(planner = planner)

	def enable_pid_planner(self, enable: bool) -> SetBool.Response:
		return self.enable_pid_planner_serv.send_request(data = enable)

	def enable_dwa_planner(self, enable: bool) -> SetBool.Response:
		return self.enable_dwa_planner_serv.send_request(data = enable)

	### arm ##################################################################

	def change_arm_control(self, velocity: bool) -> SetBool.Response:
		return self.arm_control_serv.send_request(data=velocity)

	def change_arm_speed_multiplier(self, multipler: bool):
		return
		# return self.arm_speed_multiplier_serv.send_request(value=multipler)

	def change_arm_safety_features(self, enable_arm_safety: bool) -> SetBool.Response:
		return self.arm_safety_check_serv.send_request(data=enable_arm_safety)

	### science ##############################################################

	def linear_act(self, actuator_id: int, actuator_num:int, pwm_pos: int):
		msg = UInt8MultiArray()
		msg.data = [actuator_id, actuator_num, 0, pwm_pos]

		self.pico_pub.publish(msg)

		time.sleep(0.1)

		return self.pico_feedback

	def temp_probe(self):
		temp_id = 40
		def send_msg():
			msg = UInt8MultiArray()
			msg.data = [temp_id, 0, 0, 0]
			self.pico_pub.publish(msg)
			time.sleep(0.1)
		
		send_msg()
		return self.pico_feedback
	
	def moisture_sensor(self):
		moisture_id = 41
		def send_msg():
			msg = UInt8MultiArray()
			msg.data = [41, 0, 0, 0]
			self.pico_pub.publish(msg)
			time.sleep(0.1)
		
		send_msg()

		return self.pico_feedback


	def pump(self, pump_id: int, pump_num:int, direction: int, duration: int):
		def send_msg():
			msg = UInt8MultiArray()
			msg.data = [pump_id, pump_num, direction, 0]

			self.pico_pub.publish(msg)
				
			time.sleep(duration)

			msg = UInt8MultiArray()
			msg.data = [pump_id, pump_num, 0, 0]

			self.pico_pub.publish(msg)

			time.sleep(0.1)
 

		# send_msg()
		# result = self.pico_feedback
		thread = Thread(target=send_msg)
		thread.start()
  

		# # Check if succeeded
		# if not(result == ''):
		# 	result = result if result != '' else 'No feedback recieved'
		# 	self.science_log(f'Pico feedback was: {result}')
		# 	self.science_log(f"Successfully set pump")
		# else:
		# 	self.science_log("Pump failed, pico not available")	


	@threaded_decorator
	def scoop_sample(self, collector_id: int, start: bool):
		try:
			result = self.scoop_sample_serv.send_request(collector_id = collector_id, start = start).is_successful

			if result:
				self.science_log("Successfully scooped")
			else:
				raise Exception(f"Failed to scoop")
		except Exception as e:
			QMessageBox(
				QMessageBox.Critical,
				f"Scoop Failed",
				f"Failed to scoop",
				QMessageBox.Ok,
			).exec()
			self.get_logger().error(str(e))
			self.science_log("Failed to scoop")

# TODO figure out if this is needed
	# def move_cuvette(self, cuvette_num: int) -> tuple[bool, int]:
	# 	response = self.spin_centrifuge_serv.send_request(cuvette_id = cuvette_num)
	# 	success, end = response.is_successful, response.end_position
	# 	return success, end
		# return self.move_cuvette_serv.send_request(cuvette_num = cuvette_num).value

	def spin_centrifuge(self, cuvette_id) -> tuple[bool, int]:
		response = self.spin_centrifuge_serv.send_request(cuvette_id = cuvette_id)
		success, end = response.is_successful, response.end_position
		return success, end
	

	def capture_spectrometer(self, integration_time: int) -> CollectSpectrometerData.Response:
		return self.spectrometer_serv.send_request(integration_time=integration_time)
	
	def reset_scoop(self, scoop_number: int) -> bool:
		return self.send_scoop_home.send_request(collector_id = scoop_number).is_successful
	
	def set_curr_feedback(self, msg: String):
		self.pico_feedback = msg.data
	
	def reboot_scoop(self, scoop_number:int) -> bool:
		return self.reboot_scoop_serv.send_request(collector_id = scoop_number).is_successful

	def set_sub_motor_state(self, state: bool, speed: int):
		# TODO Figure out what speed to send
		# thread = Thread(target=self.subsurface_motor_serv.send_request(state=state, speed=speed).is_successful)
		# thread.start()
		self.subsurface_motor_serv.send_request(state=state, speed=speed).is_successful
	
	def light_switch(self, on):
		msg = UInt8MultiArray()
		msg.data = [39, on, 0, 0]
		self.pico_pub.publish(msg)
	
	

	
	
		

if __name__ == "__main__":
	roslink = HeimdallRosLink()
	roslink.get_logger().debug("Starting GUI Wanderer ROS link")
	rclpy.spin()
