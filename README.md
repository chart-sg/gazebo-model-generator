# Gazebo plugin for RTLS tags

This gazebo world plugin spawns models based on ROS2 parameters

## Parameters in use:
- cgh/patient/P0/count
- cgh/patient/P1/count
- cgh/patient/P2/count
- cgh/patient/P3/count

## Paramters implemented but not in use:
- triage/patient/P0/count
- triage/patient/P1/count
- triage/patient/P2/count
- triage/patient/P3/count

## Dependencies
### Install from debian packages (on Ubuntu)

Assuming you already have some Foxy debian packages installed, install gazebo_ros_pkgs as follows:
``` bash
sudo apt install ros-foxy-gazebo-ros-pkgs
```

## Installation
```
mkdir -p $HOME/gz_plugin_ws/src; cd $HOME/gz_plugin_ws/src
git clone https://github.com/sharp-rmf/gazebo-model-generator.git
cd $HOME/gz_plugin_ws

source /opt/ros/foxy/setup.bash
colcon build
```
## Usage
You can try out the plugin by running the following:
```
source /opt/ros/foxy/setup.bash; source /source $HOME/gz_plugin_ws/install/setup.bash
ros2 launch factory plugin_tryout.launch.xml

### Then in another teminal ###
ros2 param set cgh/patient/P1/count 10
```
## Using it in your own packages 

By setting the model count in the parameters listed above, the plugin will generate ```model://patient```.