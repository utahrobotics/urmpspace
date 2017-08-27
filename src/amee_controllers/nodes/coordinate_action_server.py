#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import actionlib
import threading

from std_msgs.msg import Float32
from phidgets_ros.msg import ActuatorStatesProcessed
from amee_controllers.msg import ActuatorPosAction, ActuatorPosFeedback, ActuatorPosResult, ActuatorPosGoal, CoordinateAction, CoordinateFeedback, CoordinateResult

class CoordinateServer(object):
    """Commands the actuators to certain states by listening to commands
    and actuator states and publishing to velocity. Enforces limit positions."""
    def __init__(self):
        # These are sled safety conditions when sled is in position, the arm should be above these conditions
        sled_safety = np.array([0, 50, 100, 140, 175, 210])
        arm_safety = np.array([90, 85, 70, 57, 45, 0])
        self.safety_func = np.polyfit(sled_safety, arm_safety, 2) # get a quadratic approximation of where arm and sled should be above

        #self.lock = threading.Lock()

        # Objects to store feedback and results
        self.feedback = CoordinateFeedback()

        # Try to get the start position of the device
        try:
            self.curr_actuator_states = rospy.wait_for_message("/actuator_states/proc", ActuatorStatesProcessed, timeout=3)
            self.prev_actuator_states = self.curr_actuator_states
            self.curr_arm_pos = self.curr_actuator_states.arm
            self.curr_bucket_pos = self.curr_actuator_states.bucket
            self.curr_sled_pos = self.curr_actuator_states.sled

        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("Phidget node is not publishing!")
            self.curr_actuator_states = ActuatorStatesProcessed()
            self.prev_actuator_states = self.curr_actuator_states
            self.curr_arm_pos = 0
            self.curr_bucket_pos = 0
            self.curr_sled_pos = 0

        self.arm_client = actionlib.SimpleActionClient("arm_position_action", ActuatorPosAction)
        self.bucket_client = actionlib.SimpleActionClient("bucket_position_action", ActuatorPosAction)
        self.sled_client = actionlib.SimpleActionClient("sled_position_action", ActuatorPosAction)

        self.curr_goal = None
        self.encoders_are_publishing = False
        self.params = rospy.get_param("/control")
        self.sensor_timeout = self.params["sensor_timeout"]
        self.sensor_callback_time = rospy.Time.now() - rospy.Duration(self.sensor_timeout) # start timed out

        self.state_sub = rospy.Subscriber("/actuator_states/proc", ActuatorStatesProcessed, self.state_callback, queue_size=10)
        self.server = actionlib.SimpleActionServer("coordinate_action", CoordinateAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()


    def state_callback(self, actuator_states):
        """"Callback for actuator states topic. Gets time data is received
        Sets the current arm, bucket, and sled positions based on data"""
        self.sensor_callback_time = rospy.Time.now()
        self.curr_arm_pos = actuator_states.arm
        self.curr_bucket_pos = actuator_states.bucket
        self.curr_sled_pos = actuator_states.sled

        # rospy.logdebug(("Coordinator state callback pos: " + str(self.curr_actuator_states)))

    def publish_feedback(self, start_time, feedback_message):
        """Publish feedback for action client, including actuator positions,
         bools to indicate if the goals are done, and the runtime of the action"""
        self.feedback.arm_pos = self.curr_arm_pos
        self.feedback.bucket_pos = self.curr_bucket_pos
        self.feedback.sled_pos = self.curr_sled_pos

        # Set the done bools if the ending position falls within the goal
        rospy.logdebug("arm curr pos: %.2f  curr goal arm: %.2f", self.curr_arm_pos, self.curr_goal.arm)
        rospy.logdebug("bucket curr pos: %.2f  curr goal bucket: %.2f", self.curr_bucket_pos, self.curr_goal.bucket)
        rospy.logdebug("sled curr pos: %.2f  curr goal sled: %.2f", self.curr_sled_pos, self.curr_goal.sled)

        self.feedback.arm_done = self.arm_client.get_state() == actionlib.GoalStatus.SUCCEEDED
        self.feedback.bucket_done = self.bucket_client.get_state() == actionlib.GoalStatus.SUCCEEDED
        self.feedback.sled_done = self.sled_client.get_state() == actionlib.GoalStatus.SUCCEEDED

        self.feedback.runtime = (rospy.Time.now() - start_time).to_sec()

        self.server.publish_feedback(self.feedback)

    def publish_result(self, preempted=False, aborted=False, message=None):
        """Fill all the fields of the result message publish to the client"""
        result = CoordinateResult()
        result.total_time = self.feedback.runtime
        arm_done = self.arm_client.get_state() == actionlib.GoalStatus.SUCCEEDED
        bucket_done = self.bucket_client.get_state() == actionlib.GoalStatus.SUCCEEDED
        sled_done = self.sled_client.get_state() == actionlib.GoalStatus.SUCCEEDED

        if preempted:
            result.exit_message = "Coordinate goal cancelled by user"
            self.server.set_preempted(result=result)
        elif aborted:
            result.exit_message = "Invalid goal. Arm and sled would collide. Goal was cancelled"
            self.server.set_aborted(result=result)
        elif arm_done and bucket_done and sled_done:
            result.exit_message = "All targets were reached"
            self.server.set_succeeded(result=result)
        else:
            result.exit_message = "Positions were not reached. sry " + str(self.feedback.arm_done + self.feedback.bucket_done + self.feedback.sled_done)
            result.exit_message += " arm " if not self.feedback.arm_done else ""
            result.exit_message += " bucket " if not self.feedback.bucket_done else ""
            result.exit_message += " sled " if not self.feedback.sled_done else ""
            self.server.set_preempted(result=result)


    def goal_is_live(self):
        """Return true if any goal is still live"""
        live = False
        for client in (self.arm_client, self.bucket_client, self.sled_client):
            rospy.logdebug("Live = %d", live)
            live = live or (client.get_state() == actionlib.GoalStatus.PENDING or client.get_state() == actionlib.GoalStatus.ACTIVE)
        return live

    def arm_callback(self, feedback):
        # TODO: more stuff on this, if necessary
        rospy.logdebug(feedback)

    def check_collisions(self):
        """Check that the arm will not collide with the treads based on where
        the sled is"""
        # evaluate safety func at the current sled position
        f_sled = np.polyval(self.safety_func, self.curr_sled_pos)
        if self.curr_bucket_pos < 100:
            f_sled += 8
        # if the arm has entered the danger zone
        if self.curr_arm_pos < f_sled:
            # if sled is going forward (arm should pause. sled should go forward)
            if self.curr_sled_pos < self.curr_goal.sled:
                if not self.feedback.arm_paused:
                    self.arm_client.cancel_goal()
                    self.feedback.arm_paused = True
            # else sled is going backward (sled should pause. arm should go forward)
            else:
                if not self.feedback.sled_paused:
                    self.sled_client.cancel_goal()
                    self.feedback.sled_paused = True
        else:
            # we are in a safe zone, unpause any paused actions
            if self.feedback.arm_paused:
                self.arm_client.send_goal(ActuatorPosGoal(self.curr_goal.arm))
                self.feedback.arm_paused = False
            elif self.feedback.sled_paused:
                self.sled_client.send_goal(ActuatorPosGoal(self.curr_goal.sled))
                self.feedback.sled_paused = False



    def execute_cb(self, goal):
        """Called when a new goal is accepted."""
        start_time = rospy.Time.now()
        self.feedback.status_message = "OK"
        self.curr_goal = goal

        # If the goals start are 0.0, this indicates the actuator should not
        # move, so set the goals to their current locations
        if goal.arm == 0.0:
            goal.arm = self.curr_arm_pos
        if goal.bucket == 0.0:
            goal.bucket = self.curr_bucket_pos
        if goal.sled == 0.0:
            goal.sled = self.curr_sled_pos

        # evaluate safety_func at sled goal and see if arm is below danger zone
        f_sled = np.polyval(self.safety_func, goal.sled)
        if self.curr_arm_pos != 0.0 and goal.arm < f_sled:
            self.publish_result(aborted=True)
            return

        rospy.loginfo("Waiting for position servers to load")
        self.arm_client.wait_for_server()
        self.bucket_client.wait_for_server()
        self.sled_client.wait_for_server()

        self.arm_client.send_goal(ActuatorPosGoal(goal.arm), feedback_cb=self.arm_callback)
        self.bucket_client.send_goal(ActuatorPosGoal(goal.bucket))
        self.sled_client.send_goal(ActuatorPosGoal(goal.sled))

        r = rospy.Rate(10) # 10hz
        # Node not shutdown and at least 1 goal is still paused or live
        while not rospy.is_shutdown() and ( self.feedback.arm_paused or self.feedback.sled_paused or self.goal_is_live()):
            rospy.logdebug("goal state: %d", self.arm_client.get_state())
            if self.server.is_preempt_requested():
                # User cancelled the request
                self.arm_client.cancel_goal()
                self.bucket_client.cancel_goal()
                self.sled_client.cancel_goal()

                rospy.loginfo("Coordinate action: PREEMPTED")
                self.publish_result(preempted=True)
                return

            if (rospy.Time.now() - self.sensor_callback_time).to_sec() > self.sensor_timeout:
                self.publish_feedback(start_time, "Encoders have not published in awhile. Pausing action.")
                rospy.loginfo(self.feedback.status_message)
                self.encoders_are_publishing = False
                r.sleep()
                continue


            self.publish_feedback(start_time, "OK")
            self.encoders_are_publishing = True

            self.check_collisions()

            self.server.publish_feedback(self.feedback)
            r.sleep()

        # We have completed the node or it was shutdown
        self.publish_result()



if __name__ == '__main__':
    rospy.init_node("coordinator_action_server")
    # rospy.init_node("coordinator_action_server", log_level=rospy.DEBUG)
    server = CoordinateServer()
    rospy.spin()
