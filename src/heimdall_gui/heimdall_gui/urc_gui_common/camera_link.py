#!/usr/bin/env python3

import subprocess
import signal
from typing import Dict, Tuple, List, Callable

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSignal as Signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter

class ParamServiceClient(Node):
    def __init__(self, name, topic, type):
        super().__init__(name)
        self.cli = self.create_client(type, topic)

        self.req = type.Request()

    def send_request(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self.req, key, value)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


# Duplicated from roslink, just for adding camera subscribers
class NewSubscriber(Node):
    def __init__(self, name, topic, type, callback):
        super().__init__(name)

        self.subscription = self.create_subscription(
            type,
            topic,
            callback,
            20)
        self.subscription  # prevent unused variable warning

class CameraSubscriber(QObject):
	def __init__(self, cam_name: str, exposure_signal: Signal, image_signal: Signal(str, Image), roslink):
		super().__init__()

		self.cam_name = cam_name

		self.raw_topic = cam_name + "/image_uncompressed"
		self.status_topic = f"{cam_name}/camera_info"

		self.exposure_signal = exposure_signal
		self.image_signal = image_signal

		self.republisher_process = None

		self.sub: Node = None

		# initialize image metrics
		self.subscribed = False
		self.height = '?'
		self.width = '?'
		self.framerate_num = 5
		self.framerate_den = 1
		self.image_source = None
		self.last_timestamp = None

		self.roslink = roslink


		self.supports_manual_exposure = True

		# Service won't exist unless launched through tunnel
		DEBUG = False

		if not DEBUG:
			# Get the publisher associated with the camera
			node = self.roslink.ROSnode.get_publishers_info_by_topic(self.raw_topic)[0]
			node_name = node.node_name

			# Make a service client that can call the set_parameters service for the publisher node
			self.serv_topic = "/" + node_name + "/set_parameters"
			self.set_parameters_srv= self.roslink.make_service(self.serv_topic, SetParameters, f"{node_name}_parameters_client")
		
		topics = self.roslink.get_topics()
		has_tunnel = False
		self.has_decompressed = False

		# Check if tunnel topic exists and if decompressed topic exists
		for topic in topics:
			# if topic.find("tunnel") != -1:
			# 	has_tunnel = True
			# 	self.has_decompressed = True
			# 	break

			if topic.find("image_uncompressed") != -1:
				self.has_decompressed = True

			elif self.has_decompressed and has_tunnel:
				break

		self.roslink.get_logger().info(f"Camera name: {cam_name}")
		# Make the topic use the tcp_tunnel
		# if has_tunnel:
		# 	self.decompressed_topic = f"tcp_tunnel/{cam_name}/uncompressed"
		# 	self.roslink.get_logger().info("Found tunnel topic")

		# Make the topic be the regular uncompressed topic
		if self.has_decompressed:
			self.decompressed_topic = f'{cam_name}/image_uncompressed'
			self.roslink.get_logger().info("Found uncompressed topic")

		if self.has_decompressed:
			self.roslink.get_logger().info(self.decompressed_topic)

		else:
			self.roslink.get_logger().info(self.raw_topic)

		cam_num = int(cam_name.split("_")[1])

		# Make cams 14-17 use raw image topic
		if cam_num > 13 and cam_num < 18:
			self.has_decompressed = False

		
		self.supports_manual_exposure, self.exposure_bounds = self.get_exposure_bounds()

		

	### callbacks ############################################################
	# TODO Update this to use camera_info topic instead
	# def status_callback(self, status: Status):
	# 	self.height = status.height
	# 	self.width = status.width
	# 	self.framerate_num = status.framerate_numerator
	# 	self.framerate_den = status.framerate_denominator
	# 	self.exposure_signal.emit(self.cam_name, status.exposure)

	def image_callback(self, image: Image):
		current_timestamp = image.header.stamp.sec

		# calculate framerate for cameras that don't provide it
		# if not self.theora_webcam:
		self.height = image.height
		self.width = image.width
	
		if self.last_timestamp:
			duration = current_timestamp - self.last_timestamp
			secs = duration.to_sec()
			freq = 1 / secs
			self.framerate_num = round(freq)
			self.framerate_den = 1

		self.last_timestamp = current_timestamp
		self.image_signal.emit(self.cam_name, image)

	### decompressed #########################################################

	# display decompressed video

	def start_raw_sub(self):
		self.image_source = 'Original Publisher'

		node_name = self.raw_topic + "_sub"
		node_names = self.roslink.ROSnode.get_node_names()

		# Subscriber already exists
		if node_name in node_names:
			return
		
		# Make valid node name
		node_name = node_name.replace("/", "_")
		self.sub = NewSubscriber(node_name, self.raw_topic, Image, self.image_callback)
		self.roslink.add_subscriber(self.sub)

	def start_decompressed_sub(self):
		self.image_source = 'Decompressed Publisher'

		node_name = self.decompressed_topic + "_decomp_sub"
		node_names = self.roslink.ROSnode.get_node_names()

		# Subscriber already exists
		if node_name in node_names:
			return
		
		# Make valid node name
		node_name = node_name.replace("/", "_")
		self.sub = NewSubscriber(node_name, self.decompressed_topic, Image, self.image_callback)
		self.roslink.add_subscriber(self.sub)

	

	def stop_sub(self):
		if self.sub:
			self.sub.destroy_node()
			del self.sub
			self.sub = None

	### controls #############################################################

	def start(self):
		self.subscribed = True
		# self.start_republisher()		# raw -> republished
		# self.start_theora()				# republished -> decompressed

		# Check if decompressed, use decompressed topic if so
		# Otherwise use raw topic
		if self.has_decompressed:
			self.start_decompressed_sub()
		else:
			self.start_raw_sub()

	def stop(self):
		self.subscribed = False
		self.stop_sub()	

		self.height = '?'
		self.width = '?'
		self.framerate_num = '?'
		self.framerate_den = 1


	def restart(self):
		# Simply set the resolution to 0, and then back to normal
		success = self.set_resolution(0, 0, float(self.framerate_num) / self.framerate_den)

		success = self.set_resolution(self.height, self.width, float(self.framerate_num) / self.framerate_den)

		# if self.width and self.height and self.framerate_num and self.framerate_den:
		# 	try:
		# 		self.change_video_handler(self.width, self.height, self.framerate_num, self.framerate_den, True, True)
		if not(success):
			QMessageBox(
				QMessageBox.Critical,
				"Failed to restart camera",
				f"Could not restart camera at {self.width}x{self.height} @ {self.framerate_num}/{self.framerate_den} fps",
				QMessageBox.Ok,
			).exec()

	def get_exposure_bounds(self) -> Tuple[bool, Tuple[float]]:
		return False, (-1, -1)
		# if self.could_support_manual_exposure:
		# 	response: GetExposureBounds.Response = self.get_exposure_bounds_srv(GetExposureBounds.Request())
		# 	return response.supports_manual_exposure, response.exposure_bounds
		# return False, (-1, -1)

	# Actually sets the contrast
	def set_exposure(self, exposure: float):
		parameter = Parameter()
		parameter.name = "contrast"
		parameter.value.type = 3 # bool = 1, int = 2, float = 3, string = 4
		parameter.value.double_value = exposure

		parameter_list = [parameter]

		# if self.supports_manual_exposure:
		response: SetParameters.Response = self.set_parameters_srv.send_request(parameters = parameter_list)
		return response.results[0].successful
	
	def set_resolution(self, height, width, fps):		
		parameter = Parameter()
		parameter.name = "img_height"
		parameter.value.type = 2 # bool = 1, int = 2, float = 3, string = 4
		parameter.value.integer_value = height

		parameter2 = Parameter()
		parameter2.name = "img_width"
		parameter2.value.type = 2 # bool = 1, int = 2, float = 3, string = 4
		parameter2.value.integer_value = width

		parameter3 = Parameter()
		parameter3.name = "pub_rate_hz"
		parameter3.value.type = 3 # bool = 1, int = 2, float = 3, string = 4
		parameter3.value.double_value = float(fps)
		

		parameter_list = [parameter, parameter2, parameter3]

		# if self.supports_manual_exposure:
		response: SetParameters.Response = self.set_parameters_srv.send_request(parameters = parameter_list)
		return response.results[0].successful


class CameraFunnel(QObject):
	camera_list = Signal(dict)
	funnel_exposure_signal = Signal(str, float)

	def __init__(self, images_signal: Signal(str, Image), roslink):
		super().__init__()

		self.funnel_images_signal = images_signal
		self.funnel_images_signal.connect(self.handle_incoming_images)
		self.funnel_exposure_signal.connect(self.handle_incoming_exposures)
		self.image_slots: Dict[str, List[Callable]] = {}
		self.exposure_slots: Dict[str, List[Callable]] = {}
		self.subscribers: Dict[str, CameraSubscriber] = {}
		self.flip: Dict[str, bool] = {}

		self.flip_transform = QTransform()
		self.flip_transform.rotate(180)

		self.empty_image = QPixmap(1, 1)
		self.empty_image.fill()

		self.logger = Node(f"camera_logger").get_logger()

		self.roslink = roslink

	def set_cameras(self, camera_names: List[str], aliases: List[str]):
		for camera_name in camera_names:
			if camera_name in self.image_slots:
				continue

			self.image_slots[camera_name] = []
			self.exposure_slots[camera_name] = []
			self.flip[camera_name] = False
			self.subscribers[camera_name] = CameraSubscriber(camera_name, self.funnel_exposure_signal, self.funnel_images_signal, self.roslink)

		self.camera_list.emit(dict(zip(aliases, camera_names)))

	def subscribe(self, image_slot: Callable, exposure_slot: Callable, camera_name: str) -> Tuple[bool, bool, Tuple[float]]:
		"""
		Subscribes you to a camera feed, you must unsubscribe from previous feeds
		Returns: from a theora webcam, exposure bounds
		"""
		if camera_name in self.image_slots:
			self.image_slots[camera_name].append(image_slot)
		if camera_name in self.exposure_slots:
			self.exposure_slots[camera_name].append(exposure_slot)
		if camera_name in self.subscribers:
			subscriber = self.subscribers[camera_name]
			subscriber.start()
			
			return subscriber.supports_manual_exposure, subscriber.exposure_bounds
		return False, False, (-1, -1)

	def unsubscribe(self, image_slot: Callable, exposure_slot: Callable, camera_name: str):
		"""Unsubscribes you from a camera feed"""
		if camera_name in self.image_slots and image_slot in self.image_slots[camera_name]:
			self.image_slots[camera_name].remove(image_slot)
		if camera_name in self.exposure_slots and exposure_slot in self.exposure_slots[camera_name]:
			self.exposure_slots[camera_name].remove(exposure_slot)
		if camera_name in self.subscribers and camera_name in self.image_slots and not self.image_slots[camera_name]:
			self.subscribers[camera_name].stop()
		image_slot(self.empty_image)  # clear image

	def restart(self, camera_name: str):
		"""Restarts your camera feed"""
		if self.image_slots[camera_name] and camera_name in self.subscribers:
			self.subscribers[camera_name].restart()

	def set_exposure(self, exposure: float, camera_name: str):
		"""
		Sets the exposure of your camera feed. Note that this sets the camera to
		manual exposure, and it must be restarted to resume auto exposure.
		"""
		if self.image_slots[camera_name] and camera_name in self.subscribers:
			return self.subscribers[camera_name].set_exposure(exposure)

	def set_resolution(self, camera_name: str, height, width, fps):
		if self.image_slots[camera_name] and camera_name in self.subscribers:
			return self.subscribers[camera_name].set_resolution(height, width, fps)


	def handle_incoming_images(self, camera_name: str, raw_image: Image):
		# If no one is subscribed to this camera, don't do anything
		if not self.image_slots[camera_name]:
			return
		format = None
		if raw_image.encoding == "mono8":
			format = QImage.Format_Grayscale8
		elif raw_image.encoding == "bgr8" or raw_image.encoding == "rgb8":
			format = QImage.Format_RGB888
		else:
			self.logger.error(f"Failed to decode image, encoding {raw_image.encoding} is not recognized")
			return

		image = QImage(raw_image.data, raw_image.width, raw_image.height, format)
		if raw_image.encoding == "bgr8":
			image = image.rgbSwapped()
		if image.isNull():
			self.logger.error("Failed to decode image")
			return

		if self.flip[camera_name]:
			image = image.transformed(self.flip_transform)

		pixmap = QPixmap.fromImage(image)
		
		for func in self.image_slots[camera_name]:
			func(pixmap)



	def handle_incoming_exposures(self, camera_name: str, exposure: float):
		# If no one is subscribed to this camera, don't do anything
		if not self.image_slots[camera_name]:
			return

		for func in self.exposure_slots[camera_name]:
			func(exposure)

