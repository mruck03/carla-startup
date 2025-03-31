#!/usr/bin/env python
"""
Uses map to create waypoints to desired position
"""
import math
import sys
import threading

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy

import tf
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import time

class PedestrianAvoidance(CompatibleNode):

    """
    Generates a path using AStar for vehicle to follow based on SLAM map
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self):
        """
        Constructor
        """
        super(PedestrianAvoidance, self).__init__('pedestrian_avoidance_publisher')
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.slam_map = None #Added SLAM MAP
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.role_name = self.get_param("role_name", 'ego_vehicle')
        self.speed_command_publisher = self.new_publisher(
            Float64, "/carla/{}/speed_command".format(self.role_name),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        #Subscrive to SLAM map
        self.lidar_subscriber = self.new_subscription(
            PointCloud2,
            "/carla/{}/semantic_lidar".format(self.role_name),
            self.lidar_callback,
            qos_profile=10
        )

        # use callback to wait for ego vehicle
        self.loginfo("Waiting for ego vehicle...")
        self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)

        #Get vehicle pose:
        self.tf_listener = tf.TransformListener()
        self.lidar_frame = "{}/semantic_lidar".format(self.role_name)  # e.g., "ego_vehicle"
        self.global_frame = "map"  # Assuming "map" is the global reference frame

    def destroy(self):
        """
        Destructor
        """
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)


    def lidar_callback(self, pc):
        """
        Callback for /ego_vehicle/semantic_lidar

        Reads Lidar information and will stop if a pedestrian is close.

        :return:
        """
        # self.loginfo("Received goal, trigger rerouting...")
        # field_names = [field.name for field in pc.fields]
        # print("PointCloud2 fields:", field_names)
        points = list(pc2.read_points(pc, field_names=("x", "y", "z", "ObjTag"), skip_nans=True))
        
        # if points:
        #     print(f"First point: {points[0]}")  # Access the first point properly
        
        # Example: Check if a pedestrian (label 4, for example) is close
        for point in points:
            x, y, z, label = point
            if label == 4 and -8 < x < 8 and abs(y) < 2:  # TODO: Use speed to detect when to stop
                print("Pedestrian detected nearby! Stopping vehicle.")
                self.emergencyStop()

    def emergencyStop(self):
        stopping_speed = Float64()
        stopping_speed.data = 0.0
        self.speed_command_publisher.publish(stopping_speed)

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break

        ego_vehicle_changed = False
        if hero is None and self.ego_vehicle is not None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and \
                self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
            ego_vehicle_changed = True

        if ego_vehicle_changed:
            self.loginfo("Ego vehicle changed.")
            self.ego_vehicle = hero
        elif self.ego_vehicle:
            current_location = self.ego_vehicle.get_location()
            self.ego_vehicle_location = current_location

    def connect_to_carla(self):

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=50.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")


def main(args=None):
    """
    main function
    """
    roscomp.init('pedestrian_avoidance_publisher', args)

    waypoint_converter = None
    try:
        waypoint_converter = PedestrianAvoidance()
        waypoint_converter.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if waypoint_converter:
            waypoint_converter.destroy()
        roscomp.shutdown()


if __name__ == "__main__":
    main()
