cmake_minimum_required(VERSION 2.8.3)
project(astro)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talk src/talker.cpp)
target_link_libraries(talk ${catkin_LIBRARIES})

add_executable(drive src/drive.cpp)
target_link_libraries(drive ${catkin_LIBRARIES})