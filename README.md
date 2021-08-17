# Gazebo plugin for RTLS tags

This gazebo plugin spawns human models based on ROS2 parameters

## Parameters in use:
- cgh/patient/P0/count
- cgh/patient/P1/count
- cgh/patient/P2/count
- cgh/patient/P3/count
- triage/patient/P0/count
- triage/patient/P1/count
- triage/patient/P2/count
- triage/patient/P3/count

## Usage
By setting the values of the parameters listed above, the plugin will generate model://patient The 
build and include the libfactory.so file into your world file.