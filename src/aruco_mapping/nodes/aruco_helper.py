#!/usr/bin/env python
from __future__ import division
import rospy
import tf, tf2_ros
import actionlib
import threading
import geometry_msgs
import PyKDL



def sub_tf_stamp(a,b):
    c = geometry_msgs.msg.TransformStamped()
    c.header = a.header
    c.header.frame_id = a.header.frame_id
    c.child_frame_id = b.header.frame_id
    a_tran = a.transform.translation
    b_tran = b.transform.translation
    a_rot = a.transform.rotation
    b_rot = b.transform.rotation
    x = a_tran.x - b_tran.x
    y = a_tran.y - b_tran.y
    z = a_tran.z - b_tran.z
    c.transform.translation = geometry_msgs.msg.Vector3(x,y,z)
    a_quat = PyKDL.Rotation.Quaternion(a_rot.x, a_rot.y, a_rot.z, a_rot.w)
    b_quat = PyKDL.Rotation.Quaternion(b_rot.x, b_rot.y, b_rot.x, b_rot.w)
    
    c_quat = (a_quat * b_quat.Inverse()).GetQuaternion()
    
    c.transform.rotation = geometry_msgs.msg.Quaternion(x=c_quat[0], y=c_quat[1], z=c_quat[2], w=c_quat[3])
    return c

def main():
    rospy.init_node('aruco_helper')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster() 

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            odom_tree = tfBuffer.lookup_transform('odom', 'ZED_left_camera', rospy.Time())
            map_tree = tfBuffer.lookup_transform('map', 'camera_position', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        tf_diff = sub_tf_stamp(map_tree, odom_tree)
        br.sendTransform(tf_diff)


        rate.sleep()

if __name__ == '__main__':
    main()
