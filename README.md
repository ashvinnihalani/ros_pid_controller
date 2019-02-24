# DJI RoboMaster - Berkeley PID Controller
ROS PID Controller Movement

Instructions for use:

copy estimatedstate.msg to roborts_ws/src/RoboRTS/roborts_msgs/msg

edit cmakelists in roborts_msgs to include EstimatedState.msg

Part of it should look like:

add_message_files(

  DIRECTORY msg
  
  FILES
  
  TwistAccel.msg
  
  GimbalAngle.msg
  
  GimbalRate.msg
  
  ObstacleMsg.msg
  
  ShootInfo.msg
  
  ShootState.msg
  
  EstimatedState.msg
  
)

catkin_make

roslaunch roborts_bringup roborts_stage.launch

then run the two scripts
