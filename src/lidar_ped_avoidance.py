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
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
import rospy

import tf
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
import tf2_ros
import tf_conversions
import geometry_msgs.msg

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

        self.prev_pedestrian_positions = None
        self.filtered_pc_pub = self.new_publisher(
            PointCloud2,
            "/carla/{}/filtered_lidar".format(self.role_name),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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

        # Unpack columns

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",               # target frame
                pc.header.frame_id,  # source frame (e.g., "ego_vehicle/lidar")
                pc.header.stamp,     # time
                rospy.Duration(0.1)
            )

            pc_world = tf2_sensor_msgs.do_transform_cloud(pc, transform)

            # now you can extract points from pc_world and compare across frames
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform failed: %s", e)
            return

        points_world = np.array(list(pc2.read_points(pc_world, field_names=("x", "y", "z", "ObjTag"), skip_nans=True)))
        
        x = points_world[:, 0]
        y = points_world[:, 1]
        labels = points_world[:, 3]
        
        pedestrian_mask = (labels == 4)

        ped_pos = points_world[pedestrian_mask][:, :3]  # only x, y, z


        # Estimate motion if we have previous frame
        if self.prev_pedestrian_positions is not None and len(ped_pos) > 0 and len(self.prev_pedestrian_positions) > 0:
            dists = np.linalg.norm(ped_pos[:, None] - self.prev_pedestrian_positions[None, :], axis=2)
            min_dists = np.min(dists, axis=1)
            is_moving = min_dists > 0.02  # movement threshold in meters/frame

            dynamic_pedestrians = ped_pos[is_moving]
        else:
            dynamic_pedestrians = np.empty((0, 3))

        # Update buffer
        self.prev_pedestrian_positions = ped_pos

        dynamic_mask = np.zeros(len(points_world), dtype=bool)
        if len(dynamic_pedestrians) > 0:
            # Create a KDTree for fast spatial matching
            from scipy.spatial import cKDTree
            cloud_kd = cKDTree(points_world[:, :3])
            idxs = cloud_kd.query_ball_point(dynamic_pedestrians, r=0.2)  # tolerance radius
            for group in idxs:
                dynamic_mask[group] = True

        filtered_points = points_world[~dynamic_mask]

        filtered_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='ObjTag', offset=12, datatype=PointField.FLOAT32, count=1)
        ]


        transform_map_to_lidar = self.tf_buffer.lookup_transform(
            pc.header.frame_id,  # target: lidar frame
            "map",               # source: world/map frame
            pc.header.stamp,
            rospy.Duration(0.1)
        )

        filtered_pc_world = pc2.create_cloud(pc.header, filtered_fields, filtered_points)
        filtered_pc_world.header.frame_id = "map"
        filtered_pc_lidar = tf2_sensor_msgs.do_transform_cloud(filtered_pc_world, transform_map_to_lidar)
        self.filtered_pc_pub.publish(filtered_pc_lidar)

        dynamic_points = points_world[dynamic_mask]
        dynamic_pc_world = pc2.create_cloud(pc.header, filtered_fields, dynamic_points)
        dynamic_pc_world.header.frame_id = "map"
        dynamic_pc_lidar = tf2_sensor_msgs.do_transform_cloud(dynamic_pc_world, transform_map_to_lidar)


        dynamic_points = np.array(list(pc2.read_points(dynamic_pc_lidar, field_names=("x", "y", "z", "ObjTag"), skip_nans=True)))

        if dynamic_points.size == 0:
            return

        x = dynamic_points[:, 0]
        y = dynamic_points[:, 1]
        labels = dynamic_points[:, 3]

        # Check for nearby pedestrians (label == 4) in front of the car
        pedestrian_mask_nearby = (labels == 4) & (x > -1) & (x < 7) & (np.abs(y) < 4) #TODO: Adjust bounds based on vehicle desired speed or waypoints.

        if np.any(pedestrian_mask_nearby):
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
