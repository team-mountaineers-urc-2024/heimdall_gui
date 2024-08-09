#!/usr/bin/env python3

from heimdall_gui.urc_gui_common import RosLink

from PyQt5.QtCore import pyqtSignal as Signal

import rclpy

from std_srvs.srv import SetBool

from robot_interfaces.msg import EDWaypointList

class RoverRosLink(RosLink):
	"""Supply signals for the Qt app, originating from topic callbacks."""

	ed_waypoint_list = Signal(EDWaypointList)

	def __init__(self, executor):
		super().__init__(executor)

		self.ROSnode.declare_parameter("ed_waypoint_list_topic", "ed_waypoint_list")


		### ed ###############################################################

		ed_waypoint_list_topic = self.ROSnode.get_parameter("ed_waypoint_list_topic").value
		self.ed_waypoint_list_sub = self.make_subscriber(ed_waypoint_list_topic, EDWaypointList, self.ed_waypoint_list)
		self.ed_waypoint_list_pub = self.ROSnode.create_publisher(EDWaypointList, ed_waypoint_list_topic, 1)

		### drivetrain #######################################################

		drive_forward_service = self.ROSnode.get_parameter("drive_forward_service").value
		car_style_turning_service = self.ROSnode.get_parameter("car_style_turning_service").value

		self.drive_forward_serv = self.make_service(drive_forward_service, SetBool, "drive_forward_service_client")
		self.car_style_turning_serv = self.make_service(car_style_turning_service, SetBool, "car_style_turning_service_client")

	### ed ###################################################################

	def publish_ed_waypoints(self, ed_waypoint_list: EDWaypointList):
		self.ed_waypoint_list_pub.publish(ed_waypoint_list)

	# ### drivetrain ###########################################################

	def change_drive_direction(self, forward: bool) -> SetBool.Response:
		return self.drive_forward_serv.send_request(data=forward)

	def change_car_turning_style(self, car: bool) -> SetBool.Response:
		return self.car_style_turning_serv.send_request(data=car)
	
if __name__ == "__main__":
	roslink = RoverRosLink()
	logger = roslink.get_logger()
	logger.info("Starting GUI Rover ROS link")
	rclpy.spin()
