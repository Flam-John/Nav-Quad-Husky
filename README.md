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

<h1>Install</h1>
Create a catkin workspace with the following commands:

```
$ cd ~
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
After the the workspace is ready, clone the repository:
```

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/$USER$/UAV_simulator_ArduCopter.git
$ git clone https://github.com/HBPNeurorobotics/gazebo_dvs_plugin.git
$ git clone https://github.com/uzh-rpg/rpg_dvs_ros.git
$ git clone https://github.com/catkin/catkin_simple.git
```
Build the ardupilot_gazebo package (cross-check with the Using Gazebo Simulator with SITL documentation given above):

```
$ cd ~/catkin_ws/src/ardupilot_gazebo
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install
```

Set path of Gazebo Models/Worlds... Open up .bashrc

```
$ sudo gedit ~/.bashrc
```
Copy & Paste Following at the end of .bashrc file

```
$ source /usr/share/gazebo/setup.sh
```
```
$ export GAZEBO_MODEL_PATH=~/catkin_ws/src/UAV_simulator_ArduCopter/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
$ export GAZEBO_MODEL_PATH=~/catkin_ws/src/UAV_simulator_ArduCopter/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
$ export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/UAV_simulator_ArduCopter/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
$ export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/UAV_simulator_ArduCopter/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```

Go to the rpg_dvs_ros github repo (https://github.com/uzh-rpg/rpg_dvs_ros) and install all the dependencies according to the documentation of the package. Since the catkin_make of the ecatkin_ws is succesful you can build the simulator:

```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
$ source devel/setup.bash
$ cd ~/catkin_ws/src/UAV_simulator_ArduCopter/mavros/mavros/scripts
$ ./install_geographiclib_datasets.sh
$ cd ~/catkin_ws
```
Install is complete

Now launch a world file with a copter/rover/plane and ardupilot plugin, and it should work!

<h1>Demanding script changes</h1>

<h2>First</h2>
File: iris_coastline.launch Inside the iris_coastline package. In the line 86 you change the path of the model.sdf launched inside the script

```
<arg name="sdf_robot_file" default="/home/$USER$/catkin_ws/src/UAV_simulator_ArduCopter/ardupilot_gazebo/models/iris_with_ardupilot_and_zed_stereocamera/model.sdf"/>
```

<h2>Second</h2>
File spawn_drone.launch Inside the iris_gazebo package. In the line 13 change the path for the model.sdf file launched inside the script.

```
<arg name="sdf_robot_file" default="/home/$USER$/catkin_ws/src/UAV_simulator_ArduCopter/ardupilot_gazebo/models/iris_with_lidar/model.sdf"/>
```
<h1>Usage</h1>

Initial launch of the world
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch iris_coastline iris_coastline.launch
```
Open a second terminal and launch SITL through the scripts file in the repo:
```
$ cd ~/ardupilot/ArduCopter/
$ sim_vehicle.py --mavproxy-args="--streamrate=30" --console --map -v ArduCopter -f gazebo-iris
```
Open a third terminal and launch mavros:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch mavros apm.launch
```
These three terminals launch the sandislad world with the iris quadcopter, the SITL (both communication, telemetry, console and map) and the mavros communcations. If everyhting are launched succesfuly then you will have topics both from the ZED stereo camera.
