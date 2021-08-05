#!/bin/bash

dRootPath="../../../"
function launch_world(){
    # Launch world file
    xterm -e "pushd ${dRootPath}
    source devel/setup.bash
    export GAZEBO_MODEL_PATH=`rospack find turtlebot3_gazebo`/models/aws_warehouse/:$GAZEBO_MODEL_PATH 
    roslaunch turtlebot3_gazebo turtlebot3_world.launch" &
    sleep 5
}

function launch_slam(){
    # Perform slam gmapping
    xterm -e "pushd ${dRootPath}
    source devel/setup.bash
    roslaunch turtlebot3_gazebo turtlebot3_gmapping.launch" &
    sleep 5
}

function launch_rviz(){
    # Launch rviz 
    xterm -e "pushd ${dRootPath}
    source devel/setup.bash
    roslaunch turtlebot3_rviz_launchers view_navigation.launch" &
    sleep 5
}

function launch_teleop(){
    # Launch teleop 
    xterm -e "pushd ${dRootPath}
    source devel/setup.bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" &
    sleep 5
}

# Main starts from here
# Export Model path
launch_world
launch_slam
launch_rviz
launch_teleop


