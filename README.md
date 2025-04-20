# Self Parking Package for Carla

This project presents the development of an open-source autonomous car parking system designed for dynamic environments, addressing the gap in existing open-source solutions that often focus on holonomic path planning or simplified 2D scenarios. Unlike previous approaches, our system incorporates the non-holonomic constraints of the vehicle in a 3D scenario with moving pedestrians, providing a more realistic representation of real-world parking conditions. The system is implemented and tested in the CARLA simulation environment, with edge cases such as multiple static and patrolling pedestrians in the parking lot. 

This project was made for the EECS 568 / ROB 530 final project.

Team Members:
- Max Rucker
- Bernard Yap
- Jose Eyzaguirre
- Pranav Mallela

Project Video - 

## Requirments
This package was made with Ubuntu 20.04 in ROS Noetic.
 - Carla 0.9.13
 - Modified Carla Ros Bridge: https://github.com/mruck03/ros-bridge.git (Original - https://github.com/carla-simulator/ros-bridge)
 - SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

## Getting Started

To start using this package, follow directions for Carla Ros Bridge to create a catkin workspace. Then install both this package and the Carla ROS bridge into the src directory of the workspace
```
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/mruck03/ros-bridge.git catkin_ws/src/ros-bridge
git clone --recurse-submodules https://github.com/mruck03/carla-startup.git catkin_ws/src/carla-startup
```


Along with this, be sure to install SLAM toolbox with
```
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

Once you have this installed, make sure to source ROS and build the workspace with catkin_make

```
source /opt/ros/noetic/setup.bash
cd catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r
catkin_make
```

## How to use

### Simple Startup
To use this package, make sure Carla is running. Then, source the build and then create a roscore to use features in the package
```
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roscore
```
After this, you can run the Slam_toolbox or ORBSLAM launch file in another terminal to create a car and start the self parking loop!
```
roslaunch carla-startup slam_startup.launch
```
_(NOTE: Make sure the IP address in the slam_startup.launch file is correctly pointing to Carla)_

After this, RViz should appear and you can see the car outline, the SLAM 2D occupancy map, and the trajectory to the goal pose. If this does not load you can load the config file under the Rviz folder.

To set a new goal pose, you can use the 2D Goal pose in Rviz and select a point in the Rviz window, or publish a pose to /carla/$(arg role_name)/goal.

### Spawning Pedestrians
To spawn pedestrians in the environment, use the ROS publish command below
```
rostopic pub /carla_pedestrian_spawner/spawn_pedestrians std_msgs/Empty "{}"
```
This will spawn three pedestrians that patrol around the parking lot in a predefined path as seen in _spawn_pedestrian.py_.

## Modifying for Personal Use

### Launch Files
The slam_startup launch file is the main file that runs all the nodes needed for the parking algorithm. You can look through to see the parameters used and what programs are running. It consists of:
1. **carla-ros-bridge** - makes connection between ROS and Carla for sending commands to Carla.
2. **Carla Spawner** - Spawns vehicle and sensors into scene at given spawn_point parameter.
3. **Pointcloud to Laserscan** - Changes pointcloud given by lidar in carla to a laserscan to be used by Slam_toolbox.
4. **Rviz** - Starts up Rviz viewer.
5. **Slam_toolbox**/**Orbslam** - Slam method used for creating 2D occupancy map and localization.
6. **Path Planner** - Path planning node that uses hybrid astar to reroute to new paths as the map updates or the goal changes.
7. **Pedestrian Spawner** - Node that listens over a topic for if to spawn in or reset pedestrians.
8. **Lidar Pedestrian Avoidance** - Node that uses semantic lidar to account for dynamic pedestrians in the environment.

The main pieces of code that we introduce to this pipeline are the **Path Planner**, **Pedestrian spawner**, and **Lidar Pedestrian Avoidance**. The source code for these files can be seen in the src folder.

