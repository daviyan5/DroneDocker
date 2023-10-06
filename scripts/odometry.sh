source ~/catkin_ws/devel/setup.bash
if [ "$1" = "viz" ]; then
    roslaunch vins vins_rviz.launch
elif [ "$1" = "vins" ]; then
    rosrun vins vins_node /workspaces/odometry/configs/test_config.yaml 
elif [ "$1" = "loop" ]; then
    rosrun loop_fusion loop_fusion_node /workspaces/odometry/configs/test_config.yaml 
fi
