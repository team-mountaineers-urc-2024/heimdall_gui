#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Service, Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Log
from std_srvs.srv import Trigger
from std_msgs.msg import String, Header, Float64, Int64, Bool
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import Image, NavSatFix
from builtin_interfaces.msg import Time

from robot_interfaces.srv import GUIWaypointPath, GoToNextPoint, GoToPointSrv # type: ignore
from robot_interfaces.msg import UrcCustomPath, UrcCustomPoint # type: ignore
from ros2_aruco_interfaces.msg import ArucoMarkers
from heimdall_gui.urc_gui_common.camera_link import CameraFunnel
from heimdall_gui.urc_gui_common.widgets import MapPoint

from typing import List
import pymap3d
from PyQt5.QtCore import QObject, pyqtSignal as Signal
from copy import deepcopy


class NewSubscriber(Node):
    def __init__(self, name, topic, type, callback):
        super().__init__(name)
        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=qos.QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=1
        )
        self.subscription = self.create_subscription(
            type,
            topic,
            callback,
            qos_profile=qos_profile)
        self.subscription  # prevent unused variable warning

class NewPublisher(Node):
    def __init__(self, name, topic, type):
        super().__init__(name)
        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=qos.QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=1
        )
        self.publisher = self.create_publisher(
            type,
            topic,
            qos_profile=qos_profile)
        self.publisher  # prevent unused variable warning

# Variable service client, can be adapted for any service
class VarServiceClient(Node):
    def __init__(self, name, topic, type):
        super().__init__(name)
        self.cli = self.create_client(type, topic)
        self.topic = topic
        self.req = type.Request()
        self.type = type

    def send_request(self, **kwargs):
        count = 0
        while not(self.cli.wait_for_service(timeout_sec=0.6)) and count < 5:
            self.get_logger().info(f"Service {self.topic} not available")
            count +=1

        if count == 5:
            self.get_logger().info(f"Aborting call to {self.topic}")
            response = self.type.Response()
            return response

        for key, value in kwargs.items():
            setattr(self.req, key, value)
            
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def trigger_request(self, **kwargs):
        count = 0
        while not(self.cli.wait_for_service(timeout_sec=0.6)) and count < 5:
            self.get_logger().info(f"Service {self.topic} not available")
            count +=1

        if count == 5:
            self.get_logger().info(f"Aborting call to {self.topic}")
            response = self.type.Response()
            return response

        for key, value in kwargs.items():
            setattr(self.req, key, value)

        self.cli.call_async(self.req)

# class RosNode(Node):
#     def __init__(self, name):
#         super().__init__(name)
#         self.subscribers = []
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         for subscriber in self.subscribers:
#             rclpy.spin_once(subscriber, 0.5)

class RosLink(QObject):
    pose = Signal(PoseStamped)
    gps = Signal(NavSatFix)
    marker_list = List[UrcCustomPoint]
    state = Signal(String)
    mission_state = Signal(String)
    wp_index = Signal(String)
    seen_aruco_ids_list = Signal(ArucoMarkers)
    seen_objects_list = Signal(ArucoMarkers)
    object_reached = Signal(Bool)
    found_marker_list = Signal(list)
    current_goal = Signal(MapPoint)
    rosout = Signal(Log)
    rover_hdg_degrees = Signal(Float64)
    

    camera_funnel_signal = Signal(str, Image)

    def __init__(self, executor: MultiThreadedExecutor):
        super().__init__()
        self.ROSnode = Node("roslink_node")

        self.marker_list = [] # list of manually entered waypoints in terms of GPS
        self.enu_point_list = [] # list of manually entered waypoints in terms of ENU
        self.global_origin = None
        
        # Lists of subscribers and publishers for RosLink to iterate through
        self.subscribers:List[NewSubscriber] = []
        self.publishers:List[NewPublisher] = []
        self.executor = executor

        # Has to be declared or the rosnode won't recognize them
        self.ROSnode.declare_parameter("local_position_topic", "current_pose")
        self.ROSnode.declare_parameter("global_position_topic", "current_global_pos")
        self.ROSnode.declare_parameter("global_origin_topic", "global_origin")
        self.ROSnode.declare_parameter("local_heading_topic", "rover_compass_hdg")

        #TODO figure out if these are going to be used
        self.ROSnode.declare_parameter("state_topic", "state")
        self.ROSnode.declare_parameter("planner_status_topic", "planner_status")
        self.ROSnode.declare_parameter("select_planner_service", "select_planner")
        self.ROSnode.declare_parameter("pid_planner_name", "pid_planner")
        self.ROSnode.declare_parameter("dwa_planner_name", "dwa_planner")
        self.ROSnode.declare_parameter("pid_planner_enabled_service", "pid_planner_enabled")
        self.ROSnode.declare_parameter("dwa_planner_enabled_service", "dwa_planner_enabled")

        self.ROSnode.declare_parameter("drive_forward_service", "drive_forward")
        self.ROSnode.declare_parameter("car_style_turning_service", "car_style_turning")

        # Get topic names from passed parameters
        local_position_topic = self.ROSnode.get_parameter("local_position_topic").value
        global_position_topic = self.ROSnode.get_parameter("global_position_topic").value
        global_origin_topic = self.ROSnode.get_parameter("global_origin_topic").value
        state_topic = self.ROSnode.get_parameter("state_topic").value
        planner_status_topic = self.ROSnode.get_parameter("planner_status_topic").value
        local_heading_topic = self.ROSnode.get_parameter("local_heading_topic").value
        
    	# create subscribers
        self.pose_sub = self.make_subscriber(local_position_topic, PoseStamped, self.pose)
        self.gps_sub = self.make_subscriber(global_position_topic, NavSatFix, self.gps)
        self.global_origin_sub = self.make_subscriber(global_origin_topic, GeoPoint, self.global_origin_callback, False)
        self.state_sub = self.make_subscriber(state_topic, String, self.state)
        self.local_heading_sub = self.make_subscriber(local_heading_topic, Float64, self.rover_hdg_degrees)

        # subscribers for populating the status bar
        self.mission_state_sub = self.make_subscriber('led_color_topic', String, self.mission_state)
        self.current_wp_sub = self.make_subscriber('waypoint_index', String, self.wp_index)
        self.seen_arucos_sub = self.make_subscriber('aruco_markers', ArucoMarkers, self.seen_aruco_ids_list)
        self.seen_objects_sub = self.make_subscriber('object_poses', ArucoMarkers, self.seen_objects_list)
        self.object_reached_sub = self.make_subscriber('object_reached', Bool, self.object_reached)
    
        # create service clients
        self.send_wp_list_srv = VarServiceClient("waypoint_path_client", "send_waypoints_from_gui", GUIWaypointPath)
        self.next_wp_srv = VarServiceClient("next_point_client", "go_to_next_point", Trigger)
        # self.prev_wp_srv = VarServiceClient("prev_point_client", "go_to_prev_point", Trigger)

        self.wp_list_pub = self.ROSnode.create_publisher(UrcCustomPath, 'waypoint_list', 10)
        # used for testing previously
        # self.current_waypoint_pub = self.ROSnode.create_publisher(NavSatFix, "goal_gps", 10)
        # self.obj_type_pub = self.ROSnode.create_publisher(Int64, "target_object_id", 10)

        # rosout log messages
        self.rosout_sub = self.make_subscriber("rosout", Log, self.rosout)

        # camera funnel
        self.camera_funnel = CameraFunnel(self.camera_funnel_signal, self)

    # def update_planner_current_waypoint(self, waypoint_index: int):
    #     try:
    #         # TODO make this not confusing and wrong
    #         urc_custom_wp = self.marker_list[waypoint_index - 1] # ATTEN: this is marker list (intentional now since the planner has changed)
            
    #         goal_gps = NavSatFix(latitude=urc_custom_wp.point.point.x, longitude=urc_custom_wp.point.point.y)

    #         self.current_waypoint_pub.publish(goal_gps)
    #         self.obj_type_pub.publish(Int64(data=urc_custom_wp.aruco_id))
            
    #     except IndexError:
    #         self.get_logger().warning(f'Cannot send waypoint at index {waypoint_index}; index is out of bounds')

    def go_to_next_point(self):
        self.wp_list_pub.publish(UrcCustomPath(points=deepcopy(self.marker_list))) # publish again, one last time
        self.next_wp_srv.send_request()

    def send_waypoint_list(self, points: List[UrcCustomPoint]):
        self.wp_list_pub.publish(UrcCustomPath(points=deepcopy(self.marker_list)))

        # NOTE the block below this is old and was how it used to work
        # # if our global origin has been set, convert GPS points to ENU and send them
        # if self.global_origin:

        #     gps_points = points
        #     converted_urc_path = UrcCustomPath()

        #     for gps_point in gps_points:

        #         lat = gps_point.point.point.x # lat
        #         lon = gps_point.point.point.y # lon
        #         h = gps_point.point.point.z # alt
        #         lat0 = self.global_origin.latitude
        #         lon0 = self.global_origin.longitude
        #         h0 = self.global_origin.altitude

        #         # debug
        #         # self.get_logger().info(f'#####################')
        #         # self.get_logger().info(f'GPS Waypoint:\n\tLat:{lat}\n\tLon:{lon}\n\t{h}')
        #         # self.get_logger().info(f'GPS Origin:\n\tLat:{lat0}\n\tLon:{lon0}\n\t{h0}')
        #         # self.get_logger().info(f'#####################')
                
        #         # create PointStamped in terms of ENU
        #         e, n, u = pymap3d.geodetic2enu(lat, lon, h, lat0, lon0, h0)
        #         enu_waypoint_stamped = PointStamped()
        #         enu_waypoint_stamped.point.x = e
        #         enu_waypoint_stamped.point.y = n
        #         enu_waypoint_stamped.point.z = u # TODO might be best to just set this is 0
        #         enu_waypoint_stamped.header = gps_point.point.header

        #         # create new UrcCustomPoint and set its fields
        #         new_urc_point = UrcCustomPoint()
        #         new_urc_point.point = deepcopy(enu_waypoint_stamped)
        #         new_urc_point.location_label = deepcopy(gps_point.location_label)
        #         new_urc_point.error_radius = deepcopy(gps_point.error_radius)
        #         new_urc_point.aruco_id = deepcopy(gps_point.aruco_id)
        #         new_urc_point.aruco_id_2 = deepcopy(gps_point.aruco_id_2)

        #         # add new point to new path
        #         converted_urc_path.points.append(new_urc_point)

        #     time = Time()
        #     #TODO Figure out these values
        #     time.sec = 0
        #     time.nanosec = 0

        #     converted_urc_path.time_recieved = time
        #     self.enu_point_list = deepcopy(converted_urc_path.points)

        # else:
        #     self.get_logger().error("GPS origin is not initialized; cannot convert and send waypoint path")


    def create_custom_point(self, lat: float, lon: float, alt: float, error: float, marker_type: str, aruco_id: int, aruco_id_2: int) -> UrcCustomPoint:
        
        # set GPS data and create header
        new_waypoint = UrcCustomPoint()
        point_stamped = PointStamped()
        geom_point = Point()

        geom_point.x = lat
        geom_point.y = lon
        geom_point.z = alt

        #TODO make this update
        header = Header()
        header.stamp.sec=0
        header.stamp.nanosec=0
        header.frame_id=""

        point_stamped.header = header
        point_stamped.point = geom_point

        new_waypoint.point = point_stamped
        
        # set metadata
        new_waypoint.location_label = marker_type
        new_waypoint.error_radius = error
        new_waypoint.aruco_id = aruco_id
        new_waypoint.aruco_id_2 = aruco_id_2

        return new_waypoint
        


    def add_marker(self, lat: float, lon: float, alt: float, error: float, marker_type: str, aruco_id: int, aruco_id_2: int):
        # value = self.add_marker_srv.send_request(lat = lat, lon = lon, alt = alt, waypoint_error = error, marker_type = marker_type, aruco_id = aruco_id, aruco_id_2 = aruco_id_2)
        point = self.create_custom_point(lat, lon, alt, error, marker_type, aruco_id, 99) # FIXME hard-coded 99; trying to declutter GUI without breaking it 

        self.marker_list.append(point)

        self.send_waypoint_list(self.marker_list)


        return point
    

        
    def reorder_marker(self, row: int, new_row: int):
        to_move = self.marker_list.pop(row)
        self.marker_list.insert(new_row, to_move)
        self.send_waypoint_list(self.marker_list)
	    
    def reorder_markers(self, marker_ids: List[int]):
        pass

    def edit_marker(self, row:int, lat: float, lon: float, alt: float, error: float, marker_type: str, aruco_id: int, aruco_id_2: int, marker_id: int):
        self.marker_list.pop(row)
        edited_point = self.create_custom_point(lat, lon, 0.0, error, marker_type, aruco_id
													 ,aruco_id_2)
        self.marker_list.insert(row, edited_point)

        self.send_waypoint_list(self.marker_list)

    def remove_marker(self, row):
        self.marker_list.pop(row)
        self.send_waypoint_list(self.marker_list)

    def insert_marker(self, marker: UrcCustomPoint, row: int):
        self.marker_list.insert(row, marker)
        
        self.send_waypoint_list(self.marker_list)
    
    def clear_markers(self):
        pass
    
    def clear_found_markers(self):
        pass

    def global_origin_callback(self, global_origin: GeoPoint):
        self.global_origin = global_origin
        # self.get_logger().info(f'Global origin in the GUI set to {self.global_origin}')

    def get_logger(self):
        return self.ROSnode.get_logger()

    def add_subscriber(self, sub: NewSubscriber):
         self.subscribers.append(sub)
         self.executor.add_node(sub)
    
    def make_subscriber(self, topic: String, type, callBack, isSignal=True):
        # any slashes will result in an invalid node name; replace them
        name = (topic + "_node").replace('/', '_')
        newSub = None
        if isSignal:
            newSub = NewSubscriber(
                name,
                topic,
                type,
                lambda data: callBack.emit(data)
            )
        else:
            newSub = NewSubscriber(
                name,
                topic,
                type,
                lambda data: callBack(data)
            )
        self.subscribers.append(newSub)
        self.executor.add_node(newSub)

    def make_publisher(self, name, topic: String, type):
        newPub = NewPublisher(
                name,
                topic,
                type
            )
        self.publishers.append(newPub)
        self.executor.add_node(newPub)

    def make_service(self, serv_topic: String, serv_type: any, serv_name: String = None):
        if serv_name is None:
            serv_name = serv_topic + "_client"
        return VarServiceClient(serv_name, serv_topic, serv_type)

    def get_topics(self):
        topics = []
        for topic_type in self.ROSnode.get_topic_names_and_types():
            topics.append(topic_type[0])
        return topics
    
    
