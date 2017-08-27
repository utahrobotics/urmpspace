cmake_minimum_required(VERSION 2.8.3)
project(phidgets_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
	std_msgs
	message_generation
)

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(
  FILES
  ActuatorStates.msg
  ActuatorStatesProcessed.msg
  LoadCells.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phidgets_ros
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
