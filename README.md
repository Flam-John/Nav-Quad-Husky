# Nav-Quad-Husky
 Study and Development of Algorithms for the Cooperative Motion of a Multi-Copter Rotorcraft- Robot Vehicle Tractor Using Optical Feedback

<H1>Introduction</H1>
This Github repo features a UAV simulator integrating ArduCopter and MAVROS communication. It is developed on Ubuntu 18.04 LTS using ROS Melodic (https://wiki.ros.org/melodic) and Gazebo 9.

Before continuing with the packages explanation you should have install the ArduPilot/Copter SITL on Linux based on the thread below. Setting up SITL on Linux: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html

<H1>Packages explanation</H1>
<H2>Ardupilot_gazebo</H2>
The following plugin is a pure Gazebo plugin, so ROS is not needed to use it. You can still use ROS with Gazebo with normal gazebo-ros packages. It is based on the ArduPilot documentation (https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html). The current package is a combination of the basic ardupilot_gazebo packages:
https://github.com/khancyr/ardupilot_gazebo
https://github.com/SwiftGust/ardupilot_gazebo
with some new and modified models according to the simulator's use. Recommendation to study the Using Gazebo Simulator with SITL tutorial and then installing the present ardupilot_gazebo package in this repository.

<H2>Mavros</H2>
MAVLink extendable communication node for ROS. Modification made based on the original package (https://github.com/mavlink/mavros) in order to adapt it to our needs.

<H2>Usb_cam</H2>
A ROS driver for V4L USB cameras. In the simulator's case it can be used to extract image information from the ZED stereo camera mounted on our iris quadcopter. Source code: https://github.com/ros-drivers/usb_cam (modifications were made).

<H2>Iris_coastline</H2>
Package launching the modified vrx world with an iris quadcopter featuring a ZED stereo camera, ArduCopter and MAVROS communications.

<H2>Iris_gazebo and Iris_navigation</H2>
Packages created for a 2D navigation demo of the quadcopter featuring obstacle avoidance while using the move_base (https://wiki.ros.org/move_base) ROS package and Hokuyo Lidar.
