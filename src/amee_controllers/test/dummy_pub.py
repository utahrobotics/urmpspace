#!/usr/bin/python
import rospy
import tf
from phidgets_ros.msg import ActuatorStatesProcessed, LoadCells
from move_base_msgs.msg import MoveBaseAction

rospy.init_node('dummy_topic_pub')
states = rospy.get_param("smach_states")
mining_params = rospy.get_param("mining_params")
moves = mining_params['moves']
poses = mining_params['poses']

rospy.logwarn("NEVER run this script while the robot actuators are up. It\
is purely for testing")

act_pub = rospy.Publisher("/actuator_states/proc", ActuatorStatesProcessed, queue_size=10)

def create_fake_pose(name):
    pose = poses[name]
    arm = pose['arm']
    bucket = pose['bucket']
    sled = pose['sled']
    return ActuatorStatesProcessed(arm=arm, bucket=bucket, sled=sled)

def create_fake_odom(name):
    move = moves[name]
    position = move['position']
    deg = move['deg']

    yaw = deg * math.pi/180
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)

    # return TODO

for state in states:
    key = state.keys()[0]
    val = state[key]

    if key == 'pose':
        act_msg = create_fake_pose(val)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < 1:
            act_pub.publish(act_msg)
            rospy.sleep(0.1)


    # else:
    # rospy.sleep(0.8)
