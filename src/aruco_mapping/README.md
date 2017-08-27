# aruco_positioning_system


This package was taken from http://wiki.ros.org/aruco_mapping and modified for
our custom needs.  I think it was working before we left Florida.  There are a
few problems that need to be worked out with it, but it should be functional.


## ROS Nodes
## aruco_mapping

This node reads in camera images and tries to process these for [ArUco](http://www.uco.es/investiga/grupos/ava/node/26)
fiducial markers.  If it recognizes these, it can do some computer vision magic
to get their 3D location.  If you know the absolute position of one of these
markers, you can use it to know where you are in the world.  


This node uses the [ROS tf library][tf] extensively.  The node publishes a
transform between the `map` frame and the `camera_position` frame.


### Subscribed topics
`/zed/left/image_raw_color`<br>
Camera image using the ROS [cv_bridge]

### Published topics
`/tf`<br>
The important transform that this node publishes is the
one from `map` to `camera_position`


## aruco_helper
This is a helper node that does a pretty cool thing to be able to localize us
in the world map.  I think it works, but it hasn't been tested extensively and
was developed in Florida, right before the competition.

This allows us to localize and update the robot position in the global map by
comparing where the camera saw the aruco markers to where it exists relative
to where the robot (believes it) started.

### Subscribed topics
`/tf`<br>
- `map` to `camera_position`
- `odom` to `ZED_left_camera`

### Published topics
`/tf`<br>
- `map` to `odom`<br>
0.0, if either of the subscribed tranforms have not been heard.  After getting the initial value, it keeps republishing the last value it gets a new value.

### Parameters
- `qx, qy, qz, qz`: Quaternion angles to rotate the transformed angle by.
These are from when I was trying to fix a bug.  They don't do anything now
and should probably be taken out when the system can be tested again.


[tf]: http://wiki.ros.org/tf
[cv_bridge]: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
