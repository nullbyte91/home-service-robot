## Service Mobile Robot - Turtlebot Version 3
The goal of this project is to design a robot's environment in the gazebo, and build the mobile robot that capable of navigating to pick up and deliver virtual objects. We nedd to build the below components to complete the project.

### Map:
A Map is just a representation of an environment created from the sensor readings of the robot (for example, from the laser, among others). So just by moving the robot around the environment, you can create an awesome Map of it! In terms of ROS Navigation, this is knows as Mapping.

### Localization:
Next we need to localize the robot on that map. This map is completely useless if the robot doesn't know WHERE it is with respect to this map. This means, in order to perform a proper Navigation, the robot needs to know in which position of the Map it is located and with which orientation (that is, which direction the robot is facing) at every moment. In terms of ROS Navigation, this is known as Localization.

#### Path planning:
The Path Planning basically takes as input the current location of the robot and the position where the robot wants to go, and gives us as an output the best and fastest path in order to reach that point.


### Demo Images
![Service Robot view](https://github.com/nullbyte91/home_service_robot/blob/master/images/demo.gif)


---
**NOTE**<br>
Please switch to kinetic-devel branch if your testing this code on <br>Ubuntu 16.04</br>

---

### Setup [Ubuntu 20.04]
#### Dep Install
```bash
sudo apt-get update && sudo apt-get install ros-noetic-dwa-local-planner ros-noetic-move-base
```

#### Build
```bash
mkdir -p ~/catkin_ws/src/ && cd ~/catkin_ws/src/
git clone https://github.com/nullbyte91/home_service_robot.git
cd ../
catkin_make
chmod a+x src/home_service_robot/scripts/home_service.sh
``` 
### 🖖 Quick Start
#### Home service Robot
```bash
mkdir -p ~/catkin_ws/src/
source devel/setup.bash
cd src/home_service_robot/scripts/
bash home_service.sh
```

### 🗃 Project structure
```python
.
├── add_markers
│   ├── CMakeLists.txt
│   ├── config
│   │   └── markers.yaml
│   ├── package.xml
│   └── src
│       ├── add_markers_node_1.cpp
│       └── add_markers_node_final.cpp
├── pick_objects
│   ├── CMakeLists.txt
│   ├── config
│   │   └── pick_drop.yaml
│   ├── package.xml
│   └── src
│       ├── pick_objects_node_1.cpp
│       └── pick_objects_node_final.cpp
├── README.md
├── scripts
│   ├── add_markers.sh
│   ├── home_service.sh
│   ├── pick_object.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── slam_gmapping
│   ├── gmapping
│   │   ├── CHANGELOG.rst
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── slam_gmapping_pr2.launch
│   │   ├── nodelet_plugins.xml
│   │   ├── package.xml
│   │   ├── src
│   │   │   ├── main.cpp
│   │   │   ├── nodelet.cpp
│   │   │   ├── replay.cpp
│   │   │   ├── slam_gmapping.cpp
│   │   │   └── slam_gmapping.h
│   ├── README.md
│   └── slam_gmapping
│       ├── CHANGELOG.rst
│       ├── CMakeLists.txt
│       └── package.xml
├── turtlebot3_bringup
│   ├── 99-turtlebot3-cdc.rules
│   ├── camera_info
│   │   └── turtlebot3_rpicamera.yaml
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── includes
│   │   │   └── description.launch.xml
│   │   ├── turtlebot3_core.launch
│   │   ├── turtlebot3_lidar.launch
│   │   ├── turtlebot3_model.launch
│   │   ├── turtlebot3_realsense.launch
│   │   ├── turtlebot3_remote.launch
│   │   ├── turtlebot3_robot.launch
│   │   └── turtlebot3_rpicamera.launch
│   ├── package.xml
│   ├── scripts
│   │   └── create_udev_rules
│   └── src
│       └── turtlebot3_diagnostics.cpp
├── turtlebot3_gazebo
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── frontier_exploration.yaml
│   │   ├── gmapping_params.yaml
│   │   ├── karto_mapper_params.yaml
│   │   ├── turtlebot3_lds_2d_gazebo.lua
│   │   └── turtlebot3_lds_2d.lua
│   ├── include
│   │   └── turtlebot3_gazebo
│   │       └── turtlebot3_drive.h
│   ├── launch
│   │   ├── turtlebot3_amcl.launch
│   │   ├── turtlebot3_gmapping.launch
│   │   └── turtlebot3_world.launch
│   ├── models
│   │   ├── aws_warehouse
│   │   ├── turtlebot3_waffle_pi
│   │   │   ├── model-1_4.sdf
│   │   │   ├── model.config
│   │   │   └── model.sdf
│   │   └── turtlebot3_world
│   │       ├── meshes
│   │       │   ├── hexagon.dae
│   │       │   └── wall.dae
│   │       ├── model-1_4.sdf
│   │       ├── model.config
│   │       └── model.sdf
│   ├── package.xml
│   ├── rviz
│   │   └── turtlebot3_gazebo_model.rviz
│   ├── src
│   │   └── turtlebot3_drive.cpp
│   └── worlds
│       ├── cafe.world
│       ├── empty.world
│       ├── no_roof_small_warehouse_modified.world
│       └── no_roof_small_warehouse.world
├── turtlebot3_navigation
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── turtlebot3_amcl.launch
│   ├── maps
│   │   ├── map.pgm
│   │   └── map.yaml
│   ├── package.xml
│   └── param
│       ├── base_local_planner_params.yaml
│       ├── costmap_common_params_burger.yaml
│       ├── costmap_common_params_waffle_pi.yaml
│       ├── costmap_common_params_waffle.yaml
│       ├── dwa_local_planner_params_burger.yaml
│       ├── dwa_local_planner_params_waffle_pi.yaml
│       ├── dwa_local_planner_params_waffle.yaml
│       ├── global_costmap_params.yaml
│       ├── local_costmap_params.yaml
│       └── move_base_params.yaml
├── turtlebot3_rviz_launchers
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── view_markers.launch
│   │   ├── view_navigation.launch
│   │   └── view_slam.launch
│   ├── package.xml
│   └── rviz
│       ├── turtlebot3_gmapping.rviz
│       ├── turtlebot3_navigation_markers.rviz
│       └── turtlebot3_navigation.rviz
└── turtlebot3_teleop
    ├── CMakeLists.txt
    ├── launch
    │   └── turtlebot3_teleop_key.launch
    ├── nodes
    │   └── turtlebot3_teleop_key
    ├── package.xml
    ├── setup.py
    └── src
        └── turtlebot3_teleop
            └── __init__.py
```

### ROS Packages used in this project
<b>gmapping:</b>
We perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.

<b>turtlebot3_teleop:</b> 
With the keyboard_teleop.launch file, we can manually control a robot using keyboard commands.

<b>turtlebot3_gazebo:</b>  
Turtlebot3 robot used for the development.

<b>aws_robo_maker:</b>

Gazebo world is well suited for organizations who are building and testing robot applications for warehouse and logistics use cases.

