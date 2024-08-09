#!/usr/bin/env python3

from typing import Dict, List

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSignal as Signal
from datetime import datetime

from rclpy.node import Node
import os as os

from heimdall_gui.urc_gui_common.camera_link import CameraFunnel

class SuperCameraWidget(QLabel):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		format = QImage.Format_RGB888
		# Default image when no cameras are connected
		image = QImage(b'empty image',100, 100, format)
		self.currImage = QPixmap.fromImage(image)
		self.logNode = None

		self.adjusting_camera_list = False
		self.adjusting_exposure_slider = False
		self.current_alias = ''
		self.camera_dict = {}

		self.image_slot = self.set_image
		self.exposure_slot = self.set_exposure_slider

		self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
		self.setAlignment(Qt.AlignCenter)

		self.selector = QComboBox()
		self.selector.setSizeAdjustPolicy(self.selector.AdjustToContents)
		self.selector.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
		self.selector.addItems([''])
		self.selector.currentTextChanged.connect(self.change_camera)

		self.enable_checkbox = QCheckBox()
		self.enable_checkbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
		self.enable_checkbox.setChecked(True)
		self.enable_checkbox.toggled.connect(lambda: self.change_camera(self.selector.currentText()))

		self.restart_button = QPushButton()
		self.restart_button.setIcon(self.restart_button.style().standardIcon(QStyle.SP_BrowserReload))
		self.restart_button.clicked.connect(self.restart_camera)

		self.screenshot_button = QPushButton("Screenshot")
		self.screenshot_button.pressed.connect(lambda :self.screenshot())

		self.options_layout = QHBoxLayout()
		self.options_layout.setContentsMargins(0, 0, 0, 0)
		self.options_layout.addWidget(self.selector, alignment=Qt.AlignTop)
		self.options_layout.addWidget(self.enable_checkbox, alignment=Qt.AlignTop | Qt.AlignLeft)
		self.options_layout.addWidget(self.screenshot_button, alignment=Qt.AlignTop | Qt.AlignLeft)
		self.options_layout.addWidget(self.restart_button, alignment=Qt.AlignTop | Qt.AlignRight)


		self.exposure_slider = QSlider()
		self.exposure_slider.sliderPressed.connect(lambda: self.set_adjusting_exposure_slider(True))
		self.exposure_slider.sliderMoved.connect(self.change_camera_exposure)
		self.exposure_slider.sliderReleased.connect(lambda: self.set_adjusting_exposure_slider(False))
		self.exposure_slider.setValue(100)

		self.exposure_slider_wrapper_layout = QVBoxLayout()
		self.exposure_slider_wrapper_layout.setContentsMargins(0, 0, 0, 0)
		self.exposure_slider_wrapper_layout.addWidget(self.exposure_slider, alignment=Qt.AlignRight)

		self.layout = QVBoxLayout()
		self.layout.setContentsMargins(0, 0, 0, 0)
		self.layout.addLayout(self.options_layout)
		self.layout.addLayout(self.exposure_slider_wrapper_layout)
		self.setLayout(self.layout)

		self.update_controls(enable_webcam_controls=False, supports_manual_exposure=False, exposure_bounds=(-1, -1))

	### settings #############################################################

	def set_camera_funnel(self, funnel: CameraFunnel):
		self.funnel = funnel
		self.funnel.camera_list.connect(self.set_cameras)

	def set_cameras(self, camera_dict: Dict[str, str]):
		aliases = list(camera_dict.keys())
		self.camera_dict = camera_dict
		self.adjusting_camera_list = True
		current_index = self.selector.currentIndex()
		self.selector.clear()
		self.selector.addItems([''] + aliases)
		self.selector.setCurrentIndex(current_index)
		self.selector.adjustSize()
		self.adjusting_camera_list = False

	def set_image(self, image: QPixmap):
		"""Set the camera's current image."""
		self.currImage = image
		self.setPixmap(image.scaled(self.width(), self.height(), Qt.KeepAspectRatio))
	

	def update_controls(self, enable_webcam_controls: bool, supports_manual_exposure: bool, exposure_bounds: List[float]):
		self.restart_button.setEnabled(enable_webcam_controls)
		if True or (supports_manual_exposure and exposure_bounds != (-1, -1)):
			self.exposure_slider.setEnabled(True)
			self.exposure_slider.setRange(0, 100)#*list(map(float, (0, 1))))
		else:
			self.exposure_slider.setEnabled(False)
			self.exposure_slider.setSliderPosition(0)

	### camera feed manipulation #############################################

	def subscribe(self, camera_alias):
		supports_manual_exposure, exposure_bounds = False, (-1, -1)
		
		camera_name = self.camera_dict.get(camera_alias, None)
		if camera_name:
			supports_manual_exposure, exposure_bounds = self.funnel.subscribe(self.image_slot, self.exposure_slot, camera_name)
		self.update_controls(True, supports_manual_exposure, exposure_bounds)
		

	def unsubscribe(self, camera_alias):
		camera_name = self.camera_dict.get(camera_alias, None)
		
		if camera_name:
			self.funnel.unsubscribe(self.image_slot, self.exposure_slot, camera_name)
		

	def change_camera(self, new_camera_alias: str):
		# the currentTextChanged event will fire as set_cameras is running,
		# so this stops us from doing anything bad
		if self.adjusting_camera_list:
			return

		# unsubscribe from current alias and update to reflect selection
		self.unsubscribe(self.current_alias)
		self.current_alias = new_camera_alias

		# subscribe to new alias, if enabled
		if self.enable_checkbox.isChecked():
			self.subscribe(self.current_alias)

	def restart_camera(self):
		# the currentTextChanged event will fire as set_cameras is running,
		# so this stops us from doing anything bad
		if self.adjusting_camera_list:
			return

		# If our current camera is the blank or not set, we're not really subscribed to anything
		if self.current_alias and self.current_alias in self.camera_dict:
			self.funnel.restart(self.camera_dict[self.current_alias])
		

	def screenshot(self):
		image = self.currImage
		

		# Get screenshot directory and number of files in it
		directory = os.path.expanduser("~/.ros/screenshots")
		if not os.path.exists(directory):
			os.makedirs(directory)

		image_name = f"{directory}/{self.selector.currentText()}_{datetime.now().strftime('%m-%d-%Y %H:%M:%S')}.png"

		# Save currently displayed image
		image.save(image_name, format="png", quality=100)

		# Log the location the screenshot was saved to
		if self.logNode is None:
			self.logNode = Node("logger").get_logger()
		
		self.logNode.info(f"Saved image as '{image_name}' to: {directory}")

	### exposure #############################################################

	def set_camera_exposure(self, exposure, camera_alias):
		camera_name = self.camera_dict.get(camera_alias, None)
		if camera_name and self.enable_checkbox.isChecked():
			self.funnel.set_exposure(exposure, camera_name)

	def set_resolution(self, camera_alias, height, width, fps):
		camera_name = self.camera_dict.get(camera_alias, None)
		
		return self.funnel.set_resolution(camera_name, height, width, fps)

	def change_camera_exposure(self):
		# the currentTextChanged event will fire as set_cameras is running,
		# so this stops us from doing anything bad
		if self.adjusting_camera_list:
			return

		exposure = self.exposure_slider.sliderPosition()
		exposure = exposure / 100.0
		self.set_camera_exposure(float(exposure), self.current_alias)

	def set_adjusting_exposure_slider(self, adjusting: bool):
		self.adjusting_exposure_slider = adjusting

	def set_exposure_slider(self, exposure: float):
		if not self.adjusting_exposure_slider:
			self.exposure_slider.setSliderPosition(int(exposure))
