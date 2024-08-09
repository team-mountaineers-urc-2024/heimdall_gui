#!/usr/bin/env python3

import os
import time
import threading
from datetime import datetime
from typing import List

import pymap3d as pm
import yaml

os.environ['QT_API'] = 'pyqt5'
from pyqtlet2.leaflet import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import NavSatFix

from robot_interfaces.msg import WaypointObject, UrcCustomPoint

from heimdall_gui.urc_gui_common import RosLink

from heimdall_gui.urc_gui_common.widgets import MapPoint, MapViewer, HLine
from heimdall_gui.urc_gui_common.helpers.file_helper import file_basenames
from heimdall_gui.urc_gui_common.helpers.internet import can_access_internet
from heimdall_gui.urc_gui_common.helpers.validator import Validators, red_if_unacceptable, valid_entries
from heimdall_gui.urc_gui_common.ui_python.autonomy_edit_dialog import AutonomyEditMarkerDialog
from heimdall_gui.urc_gui_common.ui_python.optimization_dialog import Ui_optimization_dialog

from heimdall_gui.urc_gui_common import tile_scraper

### prepare yaml #############################################################
# TODO These save points to a file. Rewrite the functions to work with the UrcCustomPoint

# def geodetic_marker_list_representer(dumper: yaml.SafeDumper, markers: GeodeticMarkerList) -> yaml.nodes.MappingNode:
# 	return dumper.represent_mapping("!GeodeticMarkerList", {
# 		"markers": markers.markers,
# 		"count": markers.count,
# 	})

# def geodetic_marker_representer(dumper: yaml.SafeDumper, marker: GeodeticMarker) -> yaml.nodes.MappingNode:
# 	return dumper.represent_mapping("!GeodeticMarker", {
# 		"gps": marker.gps,
# 		"waypoint_error": marker.waypoint_error,
# 		"marker_type": marker.marker_type,
# 		"aruco_id": marker.aruco_id,
# 		"aruco_id_2": marker.aruco_id_2,
# 	})

def geopoint_representer(dumper: yaml.SafeDumper, gp: GeoPoint) -> yaml.nodes.MappingNode:
	return dumper.represent_mapping("!GeoPoint", {
		"latitude": gp.latitude,
		"longitude": gp.longitude,
		"altitude": gp.altitude,
	})

safe_dumper = yaml.SafeDumper
# safe_dumper.add_representer(GeodeticMarkerList, geodetic_marker_list_representer)
# safe_dumper.add_representer(GeodeticMarker, geodetic_marker_representer)
safe_dumper.add_representer(GeoPoint, geopoint_representer)

# def geodetic_marker_list_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> GeodeticMarkerList:
# 	return GeodeticMarkerList(**loader.construct_mapping(node))

# def geodetic_marker_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> GeodeticMarker:
# 	return GeodeticMarker(**loader.construct_mapping(node))

def geopoint_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> GeoPoint:
	return GeoPoint(**loader.construct_mapping(node))

loader = yaml.SafeLoader
# loader.add_constructor("!GeodeticMarkerList", geodetic_marker_list_constructor)
# loader.add_constructor("!GeodeticMarker", geodetic_marker_constructor)
loader.add_constructor("!GeoPoint", geopoint_constructor)

##############################################################################

autonomy_saves_dir = os.path.expanduser('~/.ros/autonomy_saves')

def clamp(value: float, lower: float, upper: float) -> float:
	return min(upper, max(value, lower))

class MapTab(QWidget):
	def __init__(self, roslink: RosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.loggernode = roslink.get_logger()
		self.fileName = None
		self.prevTime = datetime.now()
		self.currTime = 0

		self.saved_paths_directory = os.path.expanduser("~/.ros/path_recordings")

		### override functions to subclass ###

		self.map_viewer: MapViewer = self.init_map_viewer()

		# Has to be done before initializing the map
		self.init_saved_layers()

		self.markers_tab: QTabWidget = self.init_markers_tab()


		self.setLayout(self.init_layout())
		self.init_map()

		######################################

		self.last_map_click = 0
		self.roslink = roslink
		self.roslink.gps.connect(self.gps_handler)
		self.roslink.pose.connect(self.pose_handler)
		# self.roslink.marker_list.connect(self.aut_marker_list_handler)
		# self.roslink.found_marker_list.connect(self.found_marker_list_handler)

	def init_map_viewer(self):
		map_viewer = MapViewer()
		map_viewer.map.clicked.connect(lambda l: self.map_click_handler(l['latlng']))
		map_viewer.add_point_layer('Autonomy', 'blue', 'green', 'yellow')

		return map_viewer

	def init_markers_tab(self):
		self.init_aut_marker_input()
		self.init_found_marker_output()
		self.init_load_path_output()
		markers_tab = QTabWidget()
		markers_tab.addTab(self.aut_marker_input, "Autonomy")
		markers_tab.addTab(self.found_marker_output, "Found Markers")

		# Tab for loading previously tracked rover path
		markers_tab.addTab(self.load_path_output, "Load Path")
		return markers_tab

	def init_layout(self):
		map_settings = QHBoxLayout()
		map_server_label = QLabel("MapServer:")
		self.map_server_choices = QComboBox()
		self.map_server_choices.addItems(tile_scraper.MapServers.names())
		self.map_server_choices.currentIndexChanged.connect(self.update_map_server)
		map_settings.addWidget(map_server_label, 1)
		map_settings.addWidget(self.map_server_choices, 3)

		left_pane = QWidget()
		left_pane_layout = QVBoxLayout()
		left_pane_layout.addWidget(self.markers_tab)
		left_pane_layout.addLayout(map_settings)
		left_pane.setLayout(left_pane_layout)

		splitter = QSplitter()
		splitter.addWidget(left_pane)
		splitter.addWidget(self.map_viewer)
		splitter.setStretchFactor(1, 1)  # make map stretch to fill available space

		layout = QGridLayout()
		layout.addWidget(splitter)
		return layout

	def init_map(self):
		# pre-cache map tiles if possible
		if can_access_internet():
			# cache tiles for later, since there is internet now
			t = threading.Thread(target=tile_scraper.main)
			t.start()

			initial_map_server = tile_scraper.MapServers.ARCGIS_World_Imagery
		else:
			initial_map_server = tile_scraper.MapServers.ARCGIS_World_Imagery_Cache

		self.select_map_server(initial_map_server)

	def init_aut_marker_input(self):
		self.autosave_enabled = False
		self.marker_ids = []
		self.marker_list: List[UrcCustomPoint] = []
		self.selected_row = None
		self.default_selected_row = -1

		self.aut_table = QTableWidget()
		self.aut_table.setMinimumWidth(150)
		self.aut_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.aut_table.setColumnCount(6)
		self.aut_table.setHorizontalHeaderLabels(["Lat", "Lon", "Radius", "Type", "ID", "ID 2"])
		self.aut_table.setEditTriggers(QTableWidget.NoEditTriggers)
		self.aut_table.setSelectionBehavior(QTableWidget.SelectRows)
		self.aut_table.cellClicked.connect(self.aut_marker_select_handler)
		self.aut_table.currentCellChanged.connect(lambda cr, cc, pr, pc: self.aut_marker_select_handler(cr, cc))
		self.aut_table.cellDoubleClicked.connect(self.aut_marker_alter_handler)

		self.aut_lat_entry = QLineEdit()
		self.aut_lat_entry.setPlaceholderText("Latitude")
		self.aut_lat_entry.setValidator(Validators.latitude_validator)
		self.aut_lat_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_lat_entry))

		self.aut_lon_entry = QLineEdit()
		self.aut_lon_entry.setPlaceholderText("Longitude")
		self.aut_lon_entry.setValidator(Validators.longitude_validator)
		self.aut_lon_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_lon_entry))

		self.aut_radius_entry = QLineEdit()
		self.aut_radius_entry.setPlaceholderText("Radius (m)")
		self.aut_radius_entry.setValidator(Validators.radius_validator)
		self.aut_radius_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_radius_entry))

		self.aut_marker_type = QComboBox()
		self.aut_marker_type.addItems(Validators.marker_types)

		self.aut_aruco_id_entry = QLineEdit()
		self.aut_aruco_id_entry.setPlaceholderText("Aruco ID")
		self.aut_aruco_id_entry.setValidator(Validators.aruco_validator)
		self.aut_aruco_id_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_aruco_id_entry))

		self.aut_aruco_id_2_entry = QLineEdit()
		self.aut_aruco_id_2_entry.setPlaceholderText("Aruco ID 2")
		self.aut_aruco_id_2_entry.setValidator(Validators.aruco_validator)
		self.aut_aruco_id_2_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_aruco_id_2_entry))

		self.aruco_id_layout = QHBoxLayout()
		self.aruco_id_layout.addWidget(self.aut_aruco_id_entry)
		# self.aruco_id_layout.addWidget(self.aut_aruco_id_2_entry)

		hline1 = HLine()

		self.aut_insert_button = QPushButton("Insert Marker")
		self.aut_insert_button.clicked.connect(self.insert_aut_marker)

		self.aut_add_button = QPushButton("Add Marker")
		self.aut_add_button.clicked.connect(self.add_aut_marker)

		self.new_marker_layout = QHBoxLayout()
		self.new_marker_layout.addWidget(self.aut_insert_button)
		self.new_marker_layout.addWidget(self.aut_add_button)

		self.aut_clear_button = QPushButton("Clear Markers")
		# TODO self.aut_clear_button.clicked.connect(self.clear_aut_markers)

		self.go_to_next_point_button = QPushButton("Go to Next Waypoint")
		self.go_to_next_point_button.clicked.connect(self.go_to_next_point_func)

		# NOTE old stuff for sending the current waypoint to the PID planner
		self.current_waypoint_entry = QLineEdit()
		self.current_waypoint_entry.setPlaceholderText("Waypoint Index")
		
		self.update_waypoint_button = QPushButton("Update current waypoint")
		self.update_waypoint_button.clicked.connect(self.update_planner_current_waypoint)

		self.current_waypoint_layout = QHBoxLayout()
		self.current_waypoint_layout.addWidget(self.current_waypoint_entry)
		self.current_waypoint_layout.addWidget(self.update_waypoint_button)

		self.aut_optimize_button = QPushButton("Optimize Route")
		# self.aut_optimize_button.clicked.connect(self.optimize_route_handler)

		hline2 = HLine()

		self.autonomy_save_line_edit = QLineEdit()

		autonomy_save_but = QPushButton("Save")
		autonomy_save_but.pressed.connect(self.save_autonomy_markers)

		self.autonomy_load_combo = QComboBox()
		self.autonomy_load_combo.clear()
		self.autonomy_load_combo.addItems(file_basenames(autonomy_saves_dir))

		autonomy_load_but = QPushButton("Load")
		autonomy_load_but.pressed.connect(self.load_autonomy_markers)

		autonomy_del_but = QPushButton("Delete")
		autonomy_del_but.pressed.connect(self.delete_autonomy_markers)

		autonomy_saves = QGridLayout()
		autonomy_saves.addWidget(self.autonomy_save_line_edit, 0, 0, 1, 1)
		autonomy_saves.addWidget(autonomy_save_but, 1, 0, 1, 1)
		autonomy_saves.addWidget(self.autonomy_load_combo, 0, 1, 1, 2)
		autonomy_saves.addWidget(autonomy_load_but, 1, 1, 1, 1)
		autonomy_saves.addWidget(autonomy_del_but, 1, 2, 1, 1)
		autonomy_saves.setColumnStretch(0, 2)
		autonomy_saves.setColumnStretch(1, 1)
		autonomy_saves.setColumnStretch(2, 1)

		layout = QVBoxLayout()
		layout.setContentsMargins(0, 0, 0, 0)
		layout.addWidget(self.aut_table)
		layout.addWidget(self.aut_lat_entry)
		layout.addWidget(self.aut_lon_entry)
		layout.addWidget(self.aut_radius_entry)
		layout.addWidget(self.aut_marker_type)
		layout.addLayout(self.aruco_id_layout)
		layout.addWidget(hline1)
		layout.addLayout(self.new_marker_layout)
		layout.addWidget(self.go_to_next_point_button)

		# NOTE old stuff for sending the current waypoint to the PID planner
		# layout.addLayout(self.current_waypoint_layout)
		# layout.addWidget(self.aut_clear_button)
		# layout.addWidget(self.aut_optimize_button)
		layout.addWidget(hline2)
		layout.addLayout(autonomy_saves)

		self.aut_marker_input = QWidget()
		self.aut_marker_input.setLayout(layout)

	# NOTE old stuff for sending the current waypoint to the PID planner
	def update_planner_current_waypoint(self):
		self.roslink.update_planner_current_waypoint(int(self.current_waypoint_entry.text()))

	def go_to_next_point_func(self):
		gtp_confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Confirm next point",
			f"Are you sure you want to go to the next point?",
			QMessageBox.Yes | QMessageBox.No,
		)

		if gtp_confirmation_box.exec() == QMessageBox.Yes:

			if self.roslink.marker_list[-1].location_label == "intermediary":
				error_box = QMessageBox(
					QMessageBox.Warning,
					f"List cannot end with intermediary point",
					f"The last waypoint entered is an intermediary point; please enter an objective before sending",
					QMessageBox.Ok
				)
				error_box.exec()

				return

			self.roslink.go_to_next_point()

	def init_found_marker_output(self):
		self.found_marker_table = QTableWidget()
		self.found_marker_table.setMinimumWidth(150)
		self.found_marker_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.found_marker_table.setColumnCount(3)
		self.found_marker_table.setHorizontalHeaderLabels(["ID", "Lat", "Lon"])
		self.found_marker_table.setEditTriggers(QTableWidget.NoEditTriggers)
		self.found_marker_table.setSelectionBehavior(QTableWidget.SelectRows)

		self.found_marker_clear_button = QPushButton("Clear Found Markers")
		# self.found_marker_clear_button.clicked.connect(self.clear_found_markers)

		layout = QVBoxLayout()
		layout.setContentsMargins(0, 0, 0, 0)
		layout.addWidget(self.found_marker_table)
		layout.addWidget(self.found_marker_clear_button)

		self.found_marker_output = QWidget()
		self.found_marker_output.setLayout(layout)

	def init_load_path_output(self):
		layout = QVBoxLayout()

		self.saved_paths_table = QTableWidget()
		# self.saved_paths_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.saved_paths_table.setColumnCount(2)
		self.saved_paths_table.setHorizontalHeaderLabels(["Path File", "Load"])
		self.saved_paths_table.setEditTriggers(QTableWidget.NoEditTriggers)
		self.saved_paths_table.setSelectionBehavior(QTableWidget.SelectRows)
		

		folder = self.saved_paths_directory
		if not(os.path.exists(folder)):
			os.mkdir(folder)
		list_files = os.listdir(folder)
		num_files = len(list_files)

		if num_files < 1:
			no_paths = QLabel("No saved paths available")
			layout.addWidget(no_paths)
		else:
			for index, file in enumerate(list_files):
				self.saved_paths_table.insertRow(index)
				self.saved_paths_table.setItem(index, 0, QTableWidgetItem(file))


				load_button = (QCheckBox(self.saved_paths_table))
				load_button.setText("Load")
				load_button.toggled.connect(lambda checked, file=file: self.handle_save(folder + "/" + file, checked))
				
				self.saved_paths_table.setCellWidget(index, 1, load_button)

		
		layout.addWidget(self.saved_paths_table)
		self.saved_paths_table.resizeColumnToContents(0)
		self.saved_paths_table.setColumnWidth(1, 150)

		self.load_path_output = QWidget()
		self.load_path_output.setLayout(layout)

	def init_saved_layers(self):
		folder = self.saved_paths_directory
		if not(os.path.isdir(folder)):
			os.mkdir(folder)
		
		list_files = os.listdir(folder)
		num_files = len(list_files)

		if num_files < 1:
			return
		
		else:
			# Iterate through each file in the recordings directory
			for index, file in enumerate(list_files):
				# Initialize a layer for the points in the file
				with open(folder + "/" + file, "r") as f:
					# Remove header line
					header = f.readline()
					lines = f.read().splitlines()

					self.map_viewer.init_saved_path(folder + "/" + file, lines, index)


	def handle_save(self, file: str, checked: bool):
		with open(file, "r") as f:
			# Remove header line
			header = f.readline()
			lines = f.read().splitlines()
	
		if checked:
			self.map_viewer.add_saved_path(file, lines)

		else:
			self.map_viewer.remove_lines(file)
				

	def update_map_server(self, map_server_index: int):
		map_server = tile_scraper.MapServers.values()[map_server_index]
		self.map_viewer.set_map_server(map_server.tile_url, map_server.layer_count)

	def select_map_server(self, map_server: tile_scraper.MapServer):
		try:
			map_server_index = tile_scraper.MapServers.values().index(map_server)
		except ValueError:
			err_msg = f'Could not set map server because it is not available in tile_scraper.MapServers\n'\
			f'Available MapServers: {tile_scraper.MapServers.names()}\n'\
			f'Selected MapServer: {map_server.name}'
			self.loggernode.error(err_msg)
			return

		if self.map_server_choices.currentIndex() == map_server_index:
			self.update_map_server(map_server_index)
		else:
			self.map_server_choices.setCurrentIndex(map_server_index)

	def aut_marker_select_handler(self, row, col):
		self.map_viewer.set_highlighted_point('Autonomy', row)
		self.selected_row = row
		if self.marker_list:
			m = self.marker_list[0]
			self.roslink.current_goal.emit(MapPoint(m.point.point.x, m.point.point.y, 5, ''))

	def aut_marker_alter_handler(self, row, column):
		def failed_to_alter_marker():
			# TODO: popup box
			self.loggernode.warn('Failed to alter marker!')

		# prepare popup box
		cur_lat = self.aut_table.item(row, 0).text()
		cur_lon = self.aut_table.item(row, 1).text()
		cur_rad = self.aut_table.item(row, 2).text()
		cur_type = self.aut_table.item(row, 3).text()
		cur_id = self.aut_table.item(row, 4).text()
		cur_id_2 = self.aut_table.item(row, 5).text()
		dialog_box = AutonomyEditMarkerDialog(row + 1, cur_lat, cur_lon, cur_rad, cur_type, cur_id, cur_id_2)

		# handle user request
		if dialog_box.exec():
			marker_id = self.marker_ids[row]

			# user requested the marker be reordered
			if dialog_box.reorder_btn.isChecked():
				target_row = int(dialog_box.row_entry.text()) - 1
				row_count = len(self.marker_ids) - 1
				new_row = clamp(target_row, 0, row_count)
				new_following_marker_id = self.marker_ids[new_row] # id of the marker the selected marker should be reordered before

				# set row to select when marker list comes back
				self.default_selected_row = new_row

				# send request to reorder marker
				try:
					to_move = self.marker_list.pop(row)
					self.marker_list.insert(new_row, to_move)
					self.roslink.reorder_marker(row, new_row)
				
				except Exception as e:
					failed_to_alter_marker()
					self.loggernode.error("%s" %e)

			# user requested the marker have its values edited
			elif dialog_box.edit_btn.isChecked():
				# validate entry inputs
				entries = [dialog_box.lat_entry, dialog_box.lon_entry, dialog_box.radius_entry, dialog_box.aruco_entry, dialog_box.aruco_2_entry]
				if not valid_entries(entries):
					failed_to_alter_marker()
					return

				lat = float(dialog_box.lat_entry.text())
				lon = float(dialog_box.lon_entry.text())
				radius = float(dialog_box.radius_entry.text())
				marker_type = dialog_box.type_combo.currentText()
				aruco_id = int(dialog_box.aruco_entry.text())
				aruco_id_2 = int(dialog_box.aruco_2_entry.text())

				# set row to select when marker list comes back
				self.default_selected_row = row

				# send request to edit marker
				try:
					self.marker_list.pop(row)
					edited_point = self.roslink.create_custom_point(lat, lon, 0.0, radius, marker_type, aruco_id
													 ,aruco_id_2)
					self.marker_list.insert(row, edited_point)

					self.roslink.edit_marker(row, lat, lon, 0.0, radius, marker_type, aruco_id, aruco_id_2, marker_id)
				except Exception as e:
					failed_to_alter_marker()
					self.loggernode.error("%s" %e)

			# user request the marker be deleted
			elif dialog_box.del_btn.isChecked():
				# set row to select when marker list comes back
				self.default_selected_row = row - 1

				try:
					self.marker_list.pop(row)
					self.roslink.remove_marker(row)
				except Exception as e:
					failed_to_alter_marker()
					self.loggernode.error("%s" %e)

		self.disp_aut_markers(self.marker_list)

	def gps_handler(self, gps: NavSatFix):
		self.map_viewer.set_robot_position(gps.latitude, gps.longitude)

		now = datetime.now()

		# Save only every 3 seconds
		if ((now - self.prevTime).seconds > 3):
			# Set previous save to current time
			self.prevTime = now
			self.save_point(gps)


	def save_point(self, gps: NavSatFix):
		pos = f"{str(gps.longitude)},{str(gps.latitude)}\n"
		directory = self.saved_paths_directory
		if not os.path.exists(directory):
			os.makedirs(directory)

		if self.fileName is None:
			self.fileName = f"{directory}/{datetime.now().strftime('%m-%d-%Y %H:%M:%S')}.txt"
			with open(self.fileName, "a") as f:
				f.write("Lon,Lat\n")
		
		with open(self.fileName, "a") as f:
			f.write(pos)

	
	def pose_handler(self, pose: PoseStamped):
		actual_pose = pose.pose
		orientation = actual_pose.orientation
		orientation = QQuaternion(QVector4D(orientation.x, orientation.y, orientation.z, orientation.w))
		orientation = orientation.toEulerAngles()
		self.map_viewer.set_robot_rotation(-orientation.z())

	def aut_marker_list_handler(self, marker_list: List[UrcCustomPoint]):
		self.disp_aut_markers(marker_list)
		# if self.marker_list:
		# 	m = self.marker_list[0]
		# 	self.roslink.current_goal.emit(MapPoint(m.point.point.x, point.point.y, m.waypoint_error, ''))
		if self.autosave_enabled:
			self.save_autonomy_markers_to_file('autosave')
			self.autosave_enabled = False

	def disp_aut_markers(self, marker_list: List[UrcCustomPoint]):
		# clear current markers
		self.map_viewer.unhighlight_point('Autonomy')
		marker_count = len(marker_list)
		self.aut_table.setRowCount(marker_count)
		self.marker_ids = [0] * marker_count  # clear marker_ids

		# store marker_list
		self.marker_list = marker_list

		# populate markers in table
		for row, marker in enumerate(marker_list):
			marker: UrcCustomPoint
			items = [marker.point.point.x, marker.point.point.y, marker.error_radius, marker.location_label, marker.aruco_id, marker.aruco_id_2]
			for col, s in enumerate(items):
				self.aut_table.setItem(row, col, QTableWidgetItem(str(s)))
			self.marker_ids[row] = marker.point.point.x
		
		# display markers
		points = [MapPoint(m.point.point.x, m.point.point.y, m.error_radius, '') for m in marker_list]
		self.map_viewer.set_points('Autonomy', points)

		# highlight latest marker
		row_to_highlight = self.default_selected_row
		if 0 <= row_to_highlight < len(marker_list):
			self.aut_table.setCurrentCell(row_to_highlight, 0)
			self.map_viewer.set_highlighted_point('Autonomy', row_to_highlight)
		else:
			self.map_viewer.unhighlight_point('Autonomy')

	def map_click_handler(self, latlng):
		now = time.time()
		if now - self.last_map_click < 0.500:
			self.aut_lat_entry.setText(f"{latlng['lat']:.7f}")
			self.aut_lon_entry.setText(f"{latlng['lng']:.7f}")
		
		self.last_map_click = now

	def get_marker_values(self):
		# FIXME this whole function is a kludge; its a bandaid for the janky auton stack
		intermediary_marker = self.aut_marker_type.currentIndex() == 0
		gps_marker = self.aut_marker_type.currentIndex() == 1
		aruco_marker = self.aut_marker_type.currentIndex() == 2
		hammer_marker = self.aut_marker_type.currentIndex() == 3
		bottle_marker = self.aut_marker_type.currentIndex() == 4

		# default radius for intermediary and gps
		tolerance_radius = '3'

		# placeholder IDs, 
		default_aruco_id = '99'
		gps_aruco_placeholder = '9'
		hammer_aruco_placeholder = '4'
		bottle_aruco_placeholder = '6'

		# if intermediary_marker type and no entries, set no radius and default aruco id
		if intermediary_marker:
			if self.aut_radius_entry.text() == "":
				self.aut_radius_entry.setText(tolerance_radius)
			self.aut_aruco_id_entry.setText(default_aruco_id)

		# if gps type and no entries, set no radius and 0 aruco id
		if gps_marker:
			if self.aut_radius_entry.text() == "":
				self.aut_radius_entry.setText(tolerance_radius)
			self.aut_aruco_id_entry.setText(gps_aruco_placeholder)

		# if hammer set to the specified id
		if hammer_marker:
			self.aut_aruco_id_entry.setText(hammer_aruco_placeholder)

		if bottle_marker:
			self.aut_aruco_id_entry.setText(bottle_aruco_placeholder)

		# set second aruco id to 99 unconditionally; will delete from the view if I can
		self.aut_aruco_id_2_entry.setText("99")

		# validate entry inputs
		entries = [self.aut_lat_entry, self.aut_lon_entry, self.aut_radius_entry, self.aut_aruco_id_entry, self.aut_aruco_id_2_entry]
		if not valid_entries(entries):
			raise ValueError

		# get marker values
		latf = float(self.aut_lat_entry.text())
		lonf = float(self.aut_lon_entry.text())
		altf = 0.0
		radf = float(self.aut_radius_entry.text())
		marker = self.aut_marker_type.currentText()
		aruco_id = int(self.aut_aruco_id_entry.text())
		aruco_id_2 = int(self.aut_aruco_id_2_entry.text())

		# reset input values
		self.aut_lat_entry.clear()
		self.aut_lon_entry.clear()
		self.aut_marker_type.setCurrentIndex(0)
		self.aut_radius_entry.clear()
		self.aut_aruco_id_entry.clear()
		self.aut_aruco_id_2_entry.clear()

		return latf, lonf, altf, radf, marker, aruco_id, aruco_id_2

	def add_aut_marker(self):
		def failed_to_add_marker():
			self.aut_add_button.setStyleSheet("background-color: #ffff00")
			self.aut_add_button.setText("Add Marker (Failed)")

		try:
			latf, lonf, altf, radf, marker, aruco_id, aruco_id_2 = self.get_marker_values()

			# set row to select when marker list comes back (set to next row)
			self.default_selected_row = len(self.marker_ids)

			try:
				self.autosave_enabled = True
				point = self.roslink.add_marker(latf, lonf, altf, radf, marker, aruco_id, aruco_id_2)
				self.marker_list.append(point)
				self.aut_add_button.setStyleSheet("")
				self.aut_add_button.setText("Add Marker")
				self.disp_aut_markers(self.marker_list)
			except Exception as e:
				failed_to_add_marker()
				self.loggernode.error("%s" %e)
		except ValueError:
			failed_to_add_marker()

	def insert_aut_marker(self):
		def failed_to_insert_marker():
			self.aut_insert_button.setStyleSheet("background-color: #ffff00")
			self.aut_insert_button.setText("Insert Marker (Failed)")

		try:
			latf, lonf, altf, radf, marker, aruco_id, aruco_id_2 = self.get_marker_values()

			new_point = self.roslink.create_custom_point(latf, lonf, altf, radf, marker, aruco_id, aruco_id_2)

			# get selected row, if any
			selected_row = self.selected_row if self.selected_row is not None else len(self.marker_ids)

			# set row to select when marker list comes back (set to next row)
			self.default_selected_row = selected_row

			# find id of selected marker (marker to insert before)
			new_following_marker_id = self.marker_ids[selected_row] if 0 <= selected_row < len(self.marker_ids) else 0

			try:
				self.autosave_enabled = True
				self.marker_list.insert(selected_row, new_point)
				self.roslink.insert_marker(new_point, selected_row)
				self.aut_insert_button.setStyleSheet("")
				self.aut_insert_button.setText("Insert Marker")
			except Exception as e:
				failed_to_insert_marker()
				self.loggernode.error("%s" %e)
		except ValueError:
			failed_to_insert_marker()

		self.disp_aut_markers(self.marker_list)

	# def failed_to_clear_markers(self):
	# 	self.aut_clear_button.setStyleSheet("background-color: #ffff00")
	# 	self.aut_clear_button.setText("Clear Markers (Failed)")

	# def clear_aut_markers(self):
	# 	# ask user to confirm clearing markers
	# 	confirmation_box = QMessageBox(
	# 		QMessageBox.Question,
	# 		f"Clear markers?",
	# 		f"Are you sure you want to clear all markers?",
	# 		QMessageBox.Yes | QMessageBox.No,
	# 	)

	# 	# clear markers, with confirmation
	# 	if confirmation_box.exec() == QMessageBox.Yes:
	# 		try:
	# 			self.default_selected_row = -1
	# 			self.roslink.clear_markers()
	# 			self.aut_clear_button.setStyleSheet("")
	# 			self.aut_clear_button.setText("Clear Markers")
	# 		except Exception as e:
	# 			self.failed_to_clear_markers()
	# 			self.loggernode.error("%s" %e)

	def save_autonomy_markers(self):
		# get filename to save autonomy info to
		filename = self.autonomy_save_line_edit.text()
		if not filename:
			filename = 'unnamed'

		filepath = self.save_autonomy_markers_to_file(filename)

		# confirm to user that file has been saved
		QMessageBox(
			QMessageBox.Information,
			f"Autonomy Marker Info Saved",
			f"Saved autonomy marker info to file {filepath}.",
			QMessageBox.Ok,
		).exec()

	def save_autonomy_markers_to_file(self, filename):
		# write to file
		if not os.path.exists(autonomy_saves_dir):
			os.makedirs(autonomy_saves_dir)
		filepath = f'{autonomy_saves_dir}/{filename}.yaml'
		with open(filepath, 'w') as save_file:
			yaml.safe_dump(self.marker_list, save_file)

		# refresh load_combo
		self.autonomy_load_combo.clear()
		self.autonomy_load_combo.addItems(file_basenames(autonomy_saves_dir))

		return filepath

	def load_autonomy_markers(self):
		# check if a file has been specified
		filename = self.autonomy_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to load presets from.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{autonomy_saves_dir}/{filename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"File does not exist",
				f"File {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		# read autonomy marker info from file
		with open(filepath, 'r') as save_file:
			marker_list = yaml.safe_load(save_file)

		# load autonomy data
		for marker in marker_list:
			try:
				self.roslink.add_marker(marker.point.point.x, marker.gps.longitude, marker.gps.altitude, marker.waypoint_error, marker.marker_type, marker.aruco_id, marker.aruco_id_2)
			except Exception as e:
				self.failed_to_add_marker()
				self.loggernode.error("%s" %e)

		# confirm to user that file has been loaded
		QMessageBox(
			QMessageBox.Information,
			"Autonomy Marker Info Loaded",
			f"Loaded autonomy marker info from file {filepath}.",
			QMessageBox.Ok,
		).exec()

	def delete_autonomy_markers(self):
		# check if a file has been specified
		filename = self.autonomy_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to delete.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{autonomy_saves_dir}/{filename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"File does not exist",
				f"File {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		# ask user to confirm deleting file
		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Delete preset?",
			f"Are you sure you want to delete preset {filename}?",
			QMessageBox.Yes | QMessageBox.No,
		)

		# delete file, with confirmation
		if confirmation_box.exec() == QMessageBox.Yes:
			os.remove(filepath)

			# refresh load_combo
			self.autonomy_load_combo.clear()
			self.autonomy_load_combo.addItems(file_basenames(autonomy_saves_dir))

### aruco markers ############################################################

	# def found_marker_list_handler(self, marker_list: FoundMarkerList):
	# 	self.found_marker_table.setRowCount(len(marker_list))
	# 	for row, marker in enumerate(marker_list):

	# 		# get enu information of marker
	# 		aruco_id = marker.aruco_id
	# 		x = marker.marker_enu.x
	# 		y = marker.marker_enu.y
	# 		z = marker.marker_enu.z

	# 		# calculate lat/long position of marker
	# 		lat0 = self.roslink.global_origin.latitude
	# 		lon0 = self.roslink.global_origin.longitude
	# 		h0 = self.roslink.global_origin.altitude
	# 		lat, lon, h = pm.enu2geodetic(x, y, z, lat0, lon0, h0)

	# 		items = [aruco_id, lat, lon]
	# 		for col, s in enumerate(items):
	# 			self.found_marker_table.setItem(row, col, QTableWidgetItem(str(s)))

	# 		if aruco_id in self.map_viewer.aruco_markers:
	# 			self.map_viewer.aruco_markers[aruco_id].setIcon(self.map_viewer.aruco_icon)
	# 			self.map_viewer.set_marker_position(aruco_id, lat, lon)
	# 		else:
	# 			position = [lat, lon]
	# 			aruco_marker = L.marker(position)
	# 			aruco_marker.bindTooltip(f'Aruco ID: {aruco_id}')
	# 			aruco_marker.setIcon(self.map_viewer.aruco_icon)
	# 			self.map_viewer.aruco_markers_layer.addLayer(aruco_marker)
	# 			self.map_viewer.aruco_markers[aruco_id] = aruco_marker
	# 			self.map_viewer.last_moved_marker_positions[aruco_id] = position

	# def clear_found_markers(self):
	# 	try:
	# 		self.roslink.clear_found_markers()
	# 		self.found_marker_table.clear()
	# 		for aruco_marker in self.map_viewer.aruco_markers.values():
	# 			aruco_marker.setIcon(self.map_viewer.old_aruco_icon)
	# 	except Exception as e:
	# 		QMessageBox(
	# 			QMessageBox.Critical,
	# 			"Failed to clear found markers.",
	# 			f"Could not clear found markers.",
	# 			QMessageBox.Ok,
	# 		).exec()
	# 		self.loggernode.error("%s" %e)


