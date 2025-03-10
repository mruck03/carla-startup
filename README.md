# Self Parking Package for Carla

This package was made with Ubuntu 20.04 in ROS Noetic. 

## Requirments
 - Carla 0.9.13
 - Carla Ros Bridge: https://github.com/carla-simulator/ros-bridge
 - SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

## Getting Started

To start using this package, follow directions for Carla Ros Bridge to create a catkin workspace. Then install both this package and the Carla ROS bridge into the src directory of the workspace
```
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git catkin_ws/src/ros-bridge
git clone --recurse-submodules https://github.com/mruck03/self-parking-carla-ros.git
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
To use this package, make sure Carla is running. Then, source the build and then create a roscore to use features in the package
```
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roscore
```
After this, you can run the launch file in another terminal to create a car and start the self parking loop!
```
roslaunch self-parking-carla-ros slam_startup.launch
