cmake_minimum_required(VERSION 3.1)
project(espdrone_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_service_files(FILES
  EnableMotors.srv
)

add_message_files(FILES
  Altimeter.msg
  AttitudeCommand.msg
  Compass.msg
  ControllerState.msg
  HeadingCommand.msg
  HeightCommand.msg
  MotorCommand.msg
  MotorPWM.msg
  MotorStatus.msg
  PositionXYCommand.msg
  RawImu.msg
  RawMagnetic.msg
  RawRC.msg
  RC.msg
  RuddersCommand.msg
  ServoCommand.msg
  Supply.msg
  ThrustCommand.msg
  VelocityXYCommand.msg
  VelocityZCommand.msg
  YawrateCommand.msg
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
