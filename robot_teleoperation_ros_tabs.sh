 {
     gnome-terminal \
     --tab -t "roscore"          --working-directory=$HOME/ros_ws    \
     --tab -t "baxter_gazebo"    --working-directory=$HOME/ros_ws    \
     --tab -t "openni2_tracker"  --working-directory=$HOME/ros_ws    \
     --tab -t "rviz"             --working-directory=$HOME/ros_ws    \
     --tab -t "custom_source_1"  --working-directory=$HOME/ros_ws    \
     --tab -t "custom_source_2"  --working-directory=$HOME/ros_ws    \
     --tab -t "kinect"           --working-directory=$HOME/kinect/OpenNI/Platform/Linux/Bin/x64-Release
 }
