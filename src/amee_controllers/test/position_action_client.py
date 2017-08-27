#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import actionlib
import sys

from std_msgs.msg import Float32
from amee_controllers.msg import ActuatorPosAction, ActuatorPosFeedback, ActuatorPosResult, ActuatorPosGoal


def feedback_cb(feedback):
    print(feedback)


def position_client(name, pos, cancel=False):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    print(name+'_position_action')
    client = actionlib.SimpleActionClient(name+'_position_action', ActuatorPosAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waitign for server")
    client.wait_for_server()
    print("done waiting for server")
    # Creates a goal to send to the action server.
    goal = ActuatorPosGoal(pos=pos)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

    # client.feedback_cb =
    if cancel:
        decision = raw_input("Cancel Y/n?")
        if not decision.startswith('n'):
            client.cancel_goal()

    print("waiting for result...")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        try:
            name = sys.argv[1]
            pos = float(sys.argv[2])
        except:
            pos = 150.0
            name = "arm"
        cancel = len(sys.argv) < 4
        print(sys.argv)

        print("goal = "+str(pos))
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('position_action_client')
        result = position_client(name, pos, cancel)
        print(result)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    print("done")
    rospy.spin()
