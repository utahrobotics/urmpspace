#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import tf
import actionlib
import math


from amee_controllers.msg import CoordinateAction, CoordinateGoal

def feedback_cb(callback):
    pass

def execute():
    error = false
    rospy.init_node('angle_finder')
    listener = tf.TransformListener()
    rospy.loginfo("Getting Start Position")
    rospy.sleep(2)
    #get initial position
    try:
        #listener.waitForTransform('/collection_bin_robot_0', '/beacon')
        (transform, rototation) = listener.lookupTransform("/collection_bin_robot_0", "/beacon_back_left", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        transform = [0,0,0]
        rospy.logerr("Could not get initial transform")
        error = true
        pass

    startx = transform[0]
    starty = transform[1]
    rospy.loginfo("Got Start Position %f, %f", startx, starty)

    rospy.loginfo("Moving to next Position")
    #move to next position
    client = actionlib.SimpleActionClient('coordinate_action', CoordinateAction)
    client.wait_for_server()
    goal = CoordinateGoal(sled=100)
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Moved to Position")

    rospy.loginfo("Getting second Position")
    rospy.sleep(2)
    #get the transform again
    try:
        (transform2, rotation) = listener.lookupTransform('collection_bin_robot_0', 'beacon_back_left', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        transform2 = [0,0,0]
        rospy.logerr("Could not get second transform")
        error = true
        pass
    endx = transform2[0]
    endy = transform2[1]
    rospy.loginfo("Got second Position %f, %f", endx, endy)

    rospy.loginfo("Calculating Angle")
    #do math for angle using atan
    top = endy - starty
    bottom = endx - startx
    quotient = top / bottom
    angle = math.atan2(top, bottom)
    degrees = math.degrees(angle)
    rospy.loginfo("Angle Calculated")
    rospy.loginfo("Angle(radians): %f", angle)
    rospy.loginfo("Angle(degrees): %f", degrees)
    if not error:
        rospy.set_param('initial_angle', angle)



if __name__=='__main__':
    try:
        execute()
    except rospy.ROSInterruptException:
        pass
