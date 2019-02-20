# ros_pid_controller
ROS PID Controller Movement

Instructions for use
copy estimatedstate.msg to roborts_ws/src/RoboRTS/roborts_msgs/msg
edit cmakelists in roborts_msgs to include EstimatedState.msg
catkin_make
roslaunch roborts_bringup roborts_stage.launch
then run the two scripts

todo:
edit fake_publisher to publish estimated_State messages
edit position_controller to subscribe to those messages
