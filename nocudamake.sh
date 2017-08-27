#!/bin/bash

# Script for making the workspace on computers besides the Jetson (CUDA)
catkin_make -DCATKIN_BLACKLIST_PACKAGES="zed_wrapper"
#catkin_make -DCATKIN_BLACKLIST_PACKAGES="aruco"
#catkin_make -DCATKIN_BLACKLIST_PACKAGES="aruco_ros"
#catkin_make -DCATKIN_BLACKLIST_PACKAGES="aruco_mapping"
