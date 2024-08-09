#!/usr/bin/env python3
from .urc_gui_common import AppRunner, Window, CameraTab, ToolsTab , SixCameraTab, \
FourCameraTab, SettingsTab, ToolsTab, VLine
from heimdall_gui.rover_gui_common import RoverMapTab

from heimdall_gui.current_display_widget import CurrentDisplayWidget
from heimdall_gui.science import ScienceTab
from heimdall_gui.heimdall_controls import HeimdallControlsTab
from heimdall_gui.heimdall_ros_link import HeimdallRosLink

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rclpy
from rclpy.executors import MultiThreadedExecutor

from typing import List

# Runs the Ros link nodes
class RunLink(QObject):		
	def setup(self, executor: MultiThreadedExecutor):
		self.__init__()
		self.executor = executor
		
	def run(self):
		self.executor.spin()

		# self.executor.shutdown()

class RoverWindow(Window):
	def __init__(self, *args, **kwargs):
		super().__init__('WVU URC Heimdall GUI', *args, **kwargs)

		executor = MultiThreadedExecutor()
		roslink = HeimdallRosLink(executor)

		# Allows for concurrent execution of ROS nodes
		
		self.workerthread = QThread()
		

		# Run roslink node in new thread to not stop GUI execution
		self.link = RunLink()
		self.link.setup(executor)
		self.link.moveToThread(self.workerthread)

		self.workerthread.started.connect(self.link.run)


		single_camera_tab = CameraTab(roslink)
		six_camera_tab = SixCameraTab(roslink)
		four_camera_tab_1 = FourCameraTab(roslink)
		four_camera_tab_2 = FourCameraTab(roslink)
		science_tab = ScienceTab(roslink)

		super_camera_widgets = [
			single_camera_tab.camera_widget,
			six_camera_tab.main_cam,
			six_camera_tab.right_top_cam,
			six_camera_tab.right_mid_cam,
			six_camera_tab.bot_left_cam,
			six_camera_tab.bot_mid_cam,
			six_camera_tab.bot_right_cam,
			four_camera_tab_1.top_left_cam,
			four_camera_tab_1.top_right_cam,
			four_camera_tab_1.bot_left_cam,
			four_camera_tab_1.bot_right_cam,
			four_camera_tab_2.top_left_cam,
			four_camera_tab_2.top_right_cam,
			four_camera_tab_2.bot_left_cam,
			four_camera_tab_2.bot_right_cam,
			science_tab.ui.camera1,
			science_tab.ui.camera2,
			science_tab.ui.camera3,
		]

		settings_tab = SettingsTab(roslink, super_camera_widgets)
		tools_tab = ToolsTab(roslink)

		self.tabs.addTab(single_camera_tab,				'Camera')
		self.tabs.addTab(six_camera_tab,				'Six Camera')
		self.tabs.addTab(four_camera_tab_1,				'Four Camera 1')
		self.tabs.addTab(four_camera_tab_2,				'Four Camera 2')
		self.tabs.addTab(RoverMapTab(roslink),			'Map')
		self.tabs.addTab(science_tab,					'Science')
		self.tabs.addTab(HeimdallControlsTab(roslink),	'Controls')
		self.tabs.addTab(tools_tab,						'Tools')
		self.tabs.addTab(settings_tab,					'Settings')

		self.current_display = CurrentDisplayWidget(roslink)
		self.current_display_minimum_width = self.current_display.minimumSizeHint().width()

		self.splitter: QSplitter
		self.splitter.addWidget(self.current_display)
		self.splitter.setStretchFactor(0, 1)
		self.splitter.setOpaqueResize(False)
		self.splitter.setSizes((0, self.current_display_minimum_width))
		self.splitter.splitterMoved.connect(self.resize_splitter)

		self.status_bar.connect_roslink(roslink)
		self.status_bar.connect_timer(tools_tab.timer)

		self.seen_aruco_ids_label = QLabel("Seen Aruco IDs:")
		self.seen_aruco_ids_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.seen_aruco_ids_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.seen_aruco_ids_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.seen_aruco_ids_list.connect(lambda status: self.seen_aruco_ids_display_label.setText(str(status.marker_ids)))

		self.seen_objects_label = QLabel("Seen Object IDs:")
		self.seen_objects_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.seen_objects_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.seen_objects_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.seen_objects_list.connect(lambda status: self.seen_objects_display_label.setText(str(status.marker_ids)))

		self.object_reached_label = QLabel("Object Reached:")
		self.object_reached_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.object_reached_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.object_reached_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.object_reached.connect(lambda status: self.object_reached_display_label.setText(str(status.data)))

		self.mission_state_label = QLabel("Mission State:")
		self.mission_state_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.mission_state_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.mission_state_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.mission_state.connect(lambda status: self.mission_state_display_label.setText(status.data))

		self.current_wp_label = QLabel("Current Waypoint:")
		self.current_wp_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.current_wp_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.current_wp_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.wp_index.connect(lambda status: self.current_wp_display_label.setText(status.data))


		# Start roslink thread
		self.workerthread.start()

		

	def resize_splitter(self):
		tabs_width, current_display_width = self.splitter.sizes()
		total_width = tabs_width + current_display_width
		if current_display_width > self.current_display_minimum_width:
			self.splitter.setSizes((total_width - self.current_display_minimum_width, self.current_display_minimum_width))

def main():
	rclpy.init()

	app = AppRunner(RoverWindow)

	app.start()

if __name__ == '__main__':
	main()
