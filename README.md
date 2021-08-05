## Home Service Turtlebot Robot
The goal of this project is to design a robot's environment in the gazebo, and build the mobile robot that capable of navigating to pick up and deliver virtual objects. 

### Map:
A Map is just a representation of an environment created from the sensor readings of the robot (for example, from the laser, among others). So just by moving the robot around the environment, you can create an awesome Map of it! In terms of ROS Navigation, this is knows as Mapping.

### Localization:
Next we need to localize the robot on that map. This map is completely useless if the robot doesn't know WHERE it is with respect to this map. This means, in order to perform a proper Navigation, the robot needs to know in which position of the Map it is located and with which orientation (that is, which direction the robot is facing) at every moment. In terms of ROS Navigation, this is known as Localization.

#### Path planning:
The Path Planning basically takes as input the current location of the robot and the position where the robot wants to go, and gives us as an output the best and fastest path in order to reach that point.

#### Dep Install
```bash
sudo apt-get update && sudo apt-get install ros-noetic-dwa-local-planner ros-noetic-move-base
```
### 🖖 Quick Start
#### SLAM
```bash
# Launch World
export GAZEBO_MODEL_PATH=`rospack find turtlebot3_gazebo`/models/aws_warehouse/:$GAZEBO_MODEL_PATH 
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Launch SLAM gmapping
roslaunch turtlebot3_gazebo turtlebot3_gmapping.launch

# Launch RVIZ for visualizing the map
roslaunch turtlebot3_rviz_launchers view_navigation.launch

# Save the maps
rosrun map_server map_saver -f aws

# Auto script
bash scripts/test_slam.sh
```

#### Navigation
```bash

```