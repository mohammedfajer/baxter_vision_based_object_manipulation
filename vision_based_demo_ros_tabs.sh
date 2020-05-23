#!/bin/bash
echo "Running terminals"
{
    gnome-terminal \
    --tab -t "roscore"          --working-directory=$HOME/ros_ws    \
    --tab -t "baxter_gazebo"    --working-directory=$HOME/ros_ws    \
    --tab -t "enable_robot"     --working-directory=$HOME/ros_ws    \
    --tab -t "joint_controller" --working-directory=$HOME/ros_ws    \
    --tab -t "find_object_3d"   --working-directory=$HOME/ros_ws    \
    --tab -t "rviz"             --working-directory=$HOME/ros_ws    \
    --tab -t "scripts"          --working-directory=$HOME/ros_ws    \
    --tab -t "srcs"             --working-directory=$HOME/ros_ws    \
    --tab -t "utils"            --working-directory=$HOME/ros_ws    
}
