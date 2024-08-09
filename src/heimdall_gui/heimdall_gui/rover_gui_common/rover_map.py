#!/usr/bin/env python3

import time
from typing import List
import os

import yaml

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from heimdall_gui.urc_gui_common import MapPoint, MapTab, HLine
from heimdall_gui.urc_gui_common.helpers.file_helper import file_basenames
from heimdall_gui.urc_gui_common.helpers.validator import red_if_unacceptable, valid_entries

from robot_interfaces.msg import EDWaypoint, EDWaypointList

from heimdall_gui.rover_gui_common.ui_python.ed_edit_dialog import ExtremeDeliveryEditWaypointDialog
from heimdall_gui.rover_gui_common.rover_ros_link import RoverRosLink

### prepare yaml #############################################################

def EDWaypoint_representer(dumper: yaml.SafeDumper, waypoint: EDWaypoint) -> yaml.nodes.MappingNode:
	return dumper.represent_mapping("!EDWaypoint", {
		"latitude": waypoint.latitude,
		"longitude": waypoint.longitude,
		"radius": waypoint.radius,
		"name": waypoint.name,
	})

safe_dumper = yaml.SafeDumper
safe_dumper.add_representer(EDWaypoint, EDWaypoint_representer)

def EDWaypoint_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> EDWaypoint:
	return EDWaypoint(**loader.construct_mapping(node))

loader = yaml.SafeLoader
loader.add_constructor("!EDWaypoint", EDWaypoint_constructor)

##############################################################################

ed_saves_dir = os.path.expanduser('~/.ros/ed_saves')

def clamp(value: float, lower: float, upper: float) -> float:
	return min(upper, max(value, lower))

class RoverMapTab(MapTab):
	def __init__(self, roslink: RoverRosLink, *args, **kwargs):
		super().__init__(roslink, *args, **kwargs)
		self.ed_waypoints: List[EDWaypoint] = []

		self.roslink = roslink
		self.roslink.ed_waypoint_list.connect(self.ed_update_waypoints)

	def init_map_viewer(self):
		map_viewer = super().init_map_viewer()
		map_viewer.add_point_layer('ED', 'red', 'orange', 'purple')
		return map_viewer

	def init_markers_tab(self):
		waypoints_tab = super().init_markers_tab()
		self.init_ed_waypoint_input()
		waypoints_tab.insertTab(1, self.ed_waypoint_input, "DM")
		return waypoints_tab

	def init_ed_waypoint_input(self):
		self.autosave_enabled = False

		self.ed_table = QTableWidget()
		self.ed_table.setMinimumWidth(150)
		self.ed_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.ed_table.setColumnCount(4)
		self.ed_table.setHorizontalHeaderLabels(["Name", "Lat", "Lon", "Radius"])
		self.ed_table.setEditTriggers(QTableWidget.NoEditTriggers)
		self.ed_table.setSelectionBehavior(QTableWidget.SelectRows)
		self.ed_table.cellClicked.connect(self.ed_waypoint_select_handler)
		self.ed_table.currentCellChanged.connect(lambda cr, cc, pr, pc: self.ed_waypoint_select_handler(cr, cc))
		self.ed_table.cellDoubleClicked.connect(self.ed_waypoint_alter_handler)

		self.ed_name_entry = QLineEdit()
		self.ed_name_entry.setPlaceholderText("Name")
		self.ed_name_entry.textEdited.connect(lambda _: red_if_unacceptable(self.ed_name_entry))

		self.ed_lat_entry = QLineEdit()
		self.ed_lat_entry.setPlaceholderText("Latitude")
		self.ed_lat_entry.setValidator(QDoubleValidator(-90.0, 90.0, 7))
		self.ed_lat_entry.textEdited.connect(lambda _: red_if_unacceptable(self.ed_lat_entry))

		self.ed_lon_entry = QLineEdit()
		self.ed_lon_entry.setPlaceholderText("Longitude")
		self.ed_lon_entry.setValidator(QDoubleValidator(-180.0, 180.0, 7))
		self.ed_lon_entry.textEdited.connect(lambda _: red_if_unacceptable(self.ed_lon_entry))

		self.ed_radius_entry = QLineEdit()
		self.ed_radius_entry.setPlaceholderText("Radius (m)")
		self.ed_radius_entry.setValidator(QDoubleValidator(0, 1000, 2))
		self.ed_radius_entry.textEdited.connect(lambda _: red_if_unacceptable(self.ed_radius_entry))

		hline1 = HLine()

		self.ed_insert_button = QPushButton("Insert Waypoint")
		self.ed_insert_button.clicked.connect(self.insert_ed_waypoint_handler)

		self.ed_add_button = QPushButton("Add Waypoint")
		self.ed_add_button.clicked.connect(self.add_ed_waypoint_handler)

		self.new_waypoint_layout = QHBoxLayout()
		self.new_waypoint_layout.addWidget(self.ed_insert_button)
		self.new_waypoint_layout.addWidget(self.ed_add_button)

		self.ed_clear_button = QPushButton("Clear Waypoints")
		self.ed_clear_button.clicked.connect(self.clear_ed_waypoint_handler)

		hline2 = HLine()

		self.ed_save_line_edit = QLineEdit()

		ed_save_but = QPushButton("Save")
		ed_save_but.pressed.connect(self.save_ed_waypoints)

		self.ed_load_combo = QComboBox()
		self.ed_load_combo.clear()
		self.ed_load_combo.addItems(file_basenames(ed_saves_dir))

		ed_load_but = QPushButton("Load")
		ed_load_but.pressed.connect(self.load_ed_waypoints)

		ed_del_but = QPushButton("Delete")
		ed_del_but.pressed.connect(self.delete_ed_waypoints)

		ed_saves = QGridLayout()
		ed_saves.addWidget(self.ed_save_line_edit, 0, 0, 1, 1)
		ed_saves.addWidget(ed_save_but, 1, 0, 1, 1)
		ed_saves.addWidget(self.ed_load_combo, 0, 1, 1, 2)
		ed_saves.addWidget(ed_load_but, 1, 1, 1, 1)
		ed_saves.addWidget(ed_del_but, 1, 2, 1, 1)
		ed_saves.setColumnStretch(0, 2)
		ed_saves.setColumnStretch(1, 1)
		ed_saves.setColumnStretch(2, 1)

		layout = QVBoxLayout()
		layout.setContentsMargins(0, 0, 0, 0)
		layout.addWidget(self.ed_table)
		layout.addWidget(self.ed_name_entry)
		layout.addWidget(self.ed_lat_entry)
		layout.addWidget(self.ed_lon_entry)
		layout.addWidget(self.ed_radius_entry)
		layout.addWidget(hline1)
		layout.addLayout(self.new_waypoint_layout)
		layout.addWidget(self.ed_clear_button)
		layout.addWidget(hline2)
		layout.addLayout(ed_saves)

		self.ed_waypoint_input = QWidget()
		self.ed_waypoint_input.setLayout(layout)

	def map_click_handler(self, latlng):
		now = time.time()
		if now - self.last_map_click < 0.500:
			self.aut_lat_entry.setText(f"{latlng['lat']:.7f}")
			self.aut_lon_entry.setText(f"{latlng['lng']:.7f}")
			self.ed_lat_entry.setText(f"{latlng['lat']:.7f}")
			self.ed_lon_entry.setText(f"{latlng['lng']:.7f}")
		
		self.last_map_click = now

	def ed_waypoint_select_handler(self, row, col):
		if row >= 0:
			self.map_viewer.set_highlighted_point('ED', row)
			self.selected_row = row
			w = self.ed_waypoints[row]
			self.roslink.current_goal.emit(MapPoint(w.latitude, w.longitude, w.radius, w.name))

	def ed_waypoint_alter_handler(self, row, column):
		# prepare popup box
		cur_name = self.ed_table.item(row, 0).text()
		cur_lat = self.ed_table.item(row, 1).text()
		cur_lon = self.ed_table.item(row, 2).text()
		cur_rad = self.ed_table.item(row, 3).text()
		dialog_box = ExtremeDeliveryEditWaypointDialog(row + 1, cur_name, cur_lat, cur_lon, cur_rad)

		# handle user request
		if dialog_box.exec():
			# user requested the waypoint be reordered
			if dialog_box.reorder_btn.isChecked():
				target_row = int(dialog_box.row_entry.text()) - 1
				row_count = len(self.ed_waypoints) - 1
				new_row = clamp(target_row, 0, row_count)

				# set row to select when waypoint list comes back
				self.default_selected_row = new_row

				waypoint = self.ed_waypoints.pop(row)
				self.ed_waypoints.insert(new_row, waypoint)

			# user requested the waypoint have its values edited
			elif dialog_box.edit_btn.isChecked():
				# validate entry inputs
				entries = [dialog_box.name_entry, dialog_box.lat_entry, dialog_box.lon_entry, dialog_box.radius_entry]
				if not valid_entries(entries):
					return

				name = dialog_box.name_entry.text()
				lat = float(dialog_box.lat_entry.text())
				lon = float(dialog_box.lon_entry.text())
				radius = float(dialog_box.radius_entry.text())

				# set row to select when waypoint list comes back
				self.default_selected_row = row

				self.ed_waypoints[row].name = name
				self.ed_waypoints[row].latitude = lat
				self.ed_waypoints[row].longitude = lon
				self.ed_waypoints[row].radius = radius

			# user request the waypoint be deleted
			elif dialog_box.del_btn.isChecked():
				# set row to select when waypoint list comes back
				self.default_selected_row = row - 1

				self.ed_waypoints.pop(row)

			self.ed_publish_waypoints()

	def ed_publish_waypoints(self):
		ed_waypoint_list = EDWaypointList()
		ed_waypoint_list.waypoints = self.ed_waypoints
		ed_waypoint_list.count = len(self.ed_waypoints)
		self.roslink.publish_ed_waypoints(ed_waypoint_list)

	def ed_update_waypoints(self, ed_waypoint_list: EDWaypointList):
		self.ed_waypoints = ed_waypoint_list.waypoints

		if self.autosave_enabled:
			self.save_ed_waypoints_to_file('autosave')
			self.autosave_enabled = False

		self.map_viewer.unhighlight_point('ED')
		self.ed_table.setRowCount(len(self.ed_waypoints))

		# populate waypoints in table
		for row, wpoint in enumerate(self.ed_waypoints):
			items = [wpoint.name, wpoint.latitude, wpoint.longitude, wpoint.radius]
			for col, s in enumerate(items):
				self.ed_table.setItem(row, col, QTableWidgetItem(str(s)))

		# display waypoints
		self.map_viewer.set_points('ED', [MapPoint(w.latitude, w.longitude, w.radius, w.name) for w in self.ed_waypoints])

		# highlight latest waypoint
		row_to_highlight = self.default_selected_row
		if 0 <= row_to_highlight < len(self.ed_waypoints):
			self.ed_table.setCurrentCell(row_to_highlight, 0)
			self.map_viewer.set_highlighted_point('ED', row_to_highlight)

	def get_waypoint_values(self):
		# if no_waypoint type and no entries, use defaults
		if self.ed_radius_entry.text() == "":
			self.ed_radius_entry.setText("1")

		# validate entry inputs
		entries = [self.ed_name_entry, self.ed_lat_entry, self.ed_lon_entry, self.ed_radius_entry]
		if not valid_entries(entries):
			raise ValueError

		latf = float(self.ed_lat_entry.text())
		lonf = float(self.ed_lon_entry.text())
		radf = float(self.ed_radius_entry.text())
		name = self.ed_name_entry.text()

		self.ed_lat_entry.clear()
		self.ed_lon_entry.clear()
		self.ed_radius_entry.clear()
		self.ed_name_entry.clear()

		return latf, lonf, radf, name
	
	def insert_ed_waypoint_handler(self):
		def failed_to_add_waypoint():
			self.ed_insert_button.setStyleSheet("background-color: #ffff00")
			self.ed_insert_button.setText("Add Waypoint (Failed)")

		try:
			latf, lonf, radf, name = self.get_waypoint_values()

			# get selected row, if any
			selected_row = self.selected_row or len(self.ed_waypoints)

			# set row to select when waypoint list comes back (set to next row)
			self.default_selected_row = selected_row

			# find id of selected waypoint (waypoint to insert before)
			new_following_waypoint_id = selected_row if 0 <= selected_row < len(self.ed_waypoints) else 0

			self.autosave_enabled = True
			self.ed_waypoints.insert(new_following_waypoint_id, EDWaypoint(latitude=latf, longitude=lonf, radius=radf, name=name))
			self.ed_publish_waypoints()
			self.ed_insert_button.setStyleSheet("")
			self.ed_insert_button.setText("Insert Waypoint")
		except ValueError:
			failed_to_add_waypoint()

	def add_ed_waypoint_handler(self):
		def failed_to_add_waypoint():
			self.ed_add_button.setStyleSheet("background-color: #ffff00")
			self.ed_add_button.setText("Add Waypoint (Failed)")

		try:
			latf, lonf, radf, name = self.get_waypoint_values()

			# set row to select when waypoint list comes back (set to next row)
			self.default_selected_row = len(self.ed_waypoints)

			self.autosave_enabled = True
			self.ed_waypoints.append(EDWaypoint(latitude=latf, longitude=lonf, radius=radf, name=name))
			self.ed_publish_waypoints()
			self.ed_add_button.setStyleSheet("")
			self.ed_add_button.setText("Add Waypoint")
		except ValueError:
			failed_to_add_waypoint()

	def clear_ed_waypoint_handler(self):
		# ask user to confirm clearing waypoints
		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Clear Waypoints?",
			f"Are you sure you want to clear all waypoints?",
			QMessageBox.Yes | QMessageBox.No,
		)

		# clear waypoints, with confirmation
		if confirmation_box.exec() == QMessageBox.Yes:
			self.default_selected_row = -1
			self.ed_waypoints.clear()
			self.ed_publish_waypoints()

	def save_ed_waypoints(self):
		# get filename to save ed info to
		filename = self.ed_save_line_edit.text()
		if not filename:
			filename = 'unnamed'

		filepath = self.save_ed_waypoints_to_file(filename)

		# confirm to user that file has been saved
		QMessageBox(
			QMessageBox.Information,
			f"ED Waypoint Info Saved",
			f"Saved ED waypoint info to file {filepath}.",
			QMessageBox.Ok,
		).exec()

	def save_ed_waypoints_to_file(self, filename):
		# write to file
		if not os.path.exists(ed_saves_dir):
			os.makedirs(ed_saves_dir)
		filepath = f'{ed_saves_dir}/{filename}.yaml'
		with open(filepath, 'w') as save_file:
			yaml.safe_dump(self.ed_waypoints, save_file)

		# refresh load_combo
		self.ed_load_combo.clear()
		self.ed_load_combo.addItems(file_basenames(ed_saves_dir))

		return filepath

	def load_ed_waypoints(self):
		# check if a file has been specified
		filename = self.ed_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to load presets from.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{ed_saves_dir}/{filename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"File does not exist",
				f"File {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		# read ed waypoint info from file
		with open(filepath, 'r') as save_file:
			ed_waypoints = yaml.safe_load(save_file)

		# load ed data
		for map_point in ed_waypoints:
			self.ed_waypoints.append(map_point)

		self.ed_publish_waypoints()

		# confirm to user that file has been loaded
		QMessageBox(
			QMessageBox.Information,
			"ED Waypoint Info Loaded",
			f"Loaded ED waypoint info from file {filepath}.",
			QMessageBox.Ok,
		).exec()

	def delete_ed_waypoints(self):
		# check if a file has been specified
		filename = self.ed_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to delete.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{ed_saves_dir}/{filename}.yaml'
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
			self.ed_load_combo.clear()
			self.ed_load_combo.addItems(file_basenames(ed_saves_dir))
