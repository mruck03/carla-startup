#!/usr/bin/env python3

import threading
import carla
import math
import time
import random
import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetWaypoint, GetActorWaypoint
from std_msgs.msg import Empty

import rospy


class PedestrianSpawner(CompatibleNode):

    def __init__(self):
        """
        Constructor
        """
        super(PedestrianSpawner, self).__init__('pedestrian_spawner_service')
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.on_tick = None
        # initialize ros services
        self.spawn_pedestrians_subscriber = self.new_subscription(
            Empty,
            '/carla_pedestrian_spawner/spawn_pedestrians',
            self.spawn_multi_peds,  # This will be called when a message is received
            qos_profile=10
        )
        # self.destroy_pedestrians_subscriber = self.new_subscription(
        #     Empty,
        #     '/carla_pedestrian_spawner/destroy_pedestrians',
        #     self.destroy_all_peds,  # This will be called when a message is received
        #     qos_profile=10
        # )
        
        self.place_to_coords = {
            # "cornerTopLeft" : (-65.468666, 188.412048),
            # "cornerTopRight" : (-65.468666, 157.412048),
            # "middle1" :  (-55.468666, 168.412048),
            # "middle2" :  (-57.468666, 168.412048),
            # "middle3" :  (-59.468666, 180.412048),
            "dynamic" : (-52, 180),
            "static" : (-62, 174) 
        }

        self.direction = {
            "diagonalTopRight" : (1, 1),
            "diagonalTopLeft" : (-1, 1),
            "diagonalBottomRight" : (1, -1),
            "diagonalBottomLeft" : (-1, -1),
        }

        self.spawn_dynamic = rospy.get_param('spawn_dynamic', False)
        self.spawn_static = rospy.get_param('spawn_static', False)

        self.spawn_multi_peds(None)


    def destroy(self):
        """
        Destructor
        """
        self.destroy_all_peds()
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)


    def spawn_pedestrian(self, start_x, start_y):
        # Get the walker blueprints
        # 0048 for baby
        walker_bp = random.choice(self.world.get_blueprint_library().filter('walker.pedestrian.*'))
        
        # Define spawn points for the pedestrian

        ped_loc = carla.Location(x=start_x, y=start_y, z=1)

        ped_transform = carla.Transform(ped_loc, carla.Rotation())
        # Spawn the pedestrian
        walker = self.world.spawn_actor(walker_bp, ped_transform)


        return walker

    def destroy_all_peds(self):
        
        actors = self.world.get_actors()
        peds = [actor for actor in actors if actor.type_id.startswith("walker.pedestrian")]

        for ped in peds:
            ped.destroy()

    def calculate_direction(self, current_location, target_location):
        direction = carla.Vector3D(
            x=target_location.x - current_location.x,
            y=target_location.y - current_location.y,
            z=0  # Ignore vertical movement
        )
        # Normalize the direction vector
        length = math.sqrt(direction.x**2 + direction.y**2)
        if length > 0:
            direction.x /= length
            direction.y /= length
        return direction
    
    # Function to move the pedestrian between two points
    def patrol(self, pedestrian, point1, point2, speed=3.0, num_patrol=100):
        # num 
        print("Patrol Called!")
        control = carla.WalkerControl()
        control.speed = speed
        control.jump = False

        for i in range(num_patrol):
            # Move to point1
            print(f"iter {i} of patrol")
            time.sleep(1)
            current_location = pedestrian.get_location()

            direction = self.calculate_direction(current_location, point2)
            control.direction = direction
            control.speed = speed
            pedestrian.apply_control(control)

            # One bug here is if there's obstacles in the 
            # middle the pedestrian will get stuck and walk forever
            while current_location.distance(point2) > 0.5:
                time.sleep(0.1)
                current_location = pedestrian.get_location()
                direction = self.calculate_direction(current_location, point2)
                control.direction = direction
                pedestrian.apply_control(control)
                # print(direction)

            # Stop the pedestrian briefly
            control.speed = 0
            pedestrian.apply_control(control)
            time.sleep(1)

            print(current_location.distance(point1))
            # Move to point2
            current_location = pedestrian.get_location()
            direction = self.calculate_direction(current_location, point1)
            control.direction = direction
            control.speed = speed
            pedestrian.apply_control(control)

            while current_location.distance(point1) > 0.5:
                time.sleep(0.1)
                current_location = pedestrian.get_location()
                direction = self.calculate_direction(current_location, point2)
                control.direction = direction
                pedestrian.apply_control(control)

            control.speed = 0
            pedestrian.apply_control(control)
            time.sleep(1)

    def spawn_multi_peds(self, msg):

        self.destroy_all_peds()
        # cornerTopLeft = self.place_to_coords["cornerTopLeft"]
        # # Spawn the pedestrian at the top left and make it move to the bottom right
        # pedestrian = self.spawn_pedestrian(cornerTopLeft[0], cornerTopLeft[1])
        # point1 = carla.Location(x=cornerTopLeft[0], y=cornerTopLeft[1], z=1)
        # point2 = carla.Location(x=cornerTopLeft[0] + 20, y=cornerTopLeft[1] - 20, z=1)
        # patrol_thread = threading.Thread(target=self.patrol, args=(pedestrian, point1, point2, 2.3))
        # patrol_thread.daemon = True 
        # patrol_thread.start()

        # cornerTopRight = self.place_to_coords["cornerTopRight"]
        # # Spawn the pedestrian at the top Right and make it move to the bottom left
        # pedestrian = self.spawn_pedestrian(cornerTopRight[0], cornerTopRight[1])
        # point1 = carla.Location(x=cornerTopRight[0], y=cornerTopRight[1], z=1)
        # point2 = carla.Location(x=cornerTopRight[0] + 15, y=cornerTopRight[1] + 15, z=1)
        # patrol_thread = threading.Thread(target=self.patrol, args=(pedestrian, point1, point2, 1.5))
        # patrol_thread.daemon = True 
        # patrol_thread.start()

        # middle = self.place_to_coords["middle1"]
        # # Spawn the pedestrian at the center and make it move left and right
        # pedestrian = self.spawn_pedestrian(middle[0], middle[1])
        # point1 = carla.Location(x=middle[0], y=middle[1], z=1)
        # point2 = carla.Location(x=middle[0], y=middle[1] - 10, z=1)
        # patrol_thread = threading.Thread(target=self.patrol, args=(pedestrian, point1, point2, 1.5))
        # patrol_thread.daemon = True 
        # patrol_thread.start()

        if self.spawn_dynamic:
            middle = self.place_to_coords["dynamic"]
            # Spawn the pedestrian at the center and make it move left and right
            pedestrian = self.spawn_pedestrian(middle[0], middle[1])
            point1 = carla.Location(x=middle[0], y=middle[1], z=1)
            point2 = carla.Location(x=middle[0]-20, y=middle[1]-2, z=1)
            patrol_thread = threading.Thread(target=self.patrol, args=(pedestrian, point1, point2, 3))
            patrol_thread.daemon = True 
            patrol_thread.start()

        if self.spawn_static:
            middle = self.place_to_coords["static"]
            pedestrian = self.spawn_pedestrian(middle[0], middle[1])

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
    roscomp.init('carla_pedestrian_spawner', args)

    spawner = None
    try:
        spawner = PedestrianSpawner()
        spawner.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if spawner:
            spawner.destroy()
        roscomp.shutdown()


if __name__ == "__main__":
    main()