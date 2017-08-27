#!/usr/bin/env python
from __future__ import division
from Phidgets.Devices.InterfaceKit import InterfaceKit
from Phidgets.Devices.Encoder import Encoder
from Phidgets.Devices.Bridge import Bridge, BridgeGain
#ROS imports
import rospy
from std_msgs.msg import Header
from phidgets_ros.msg import ActuatorStates, ActuatorStatesProcessed
from pololu_ros.msg import LimitSwitch

# import threading

# TODO: see what the encoder positions are reading in order to setup the limit_callback and such

class PhidgetNode(object):
    """Node for polling InterfaceKit, Encoder Board, and Wheatstone Bridge"""
    def __init__(self):
        """Open InterfaceKit and Encoder devices and initialize ros node"""
        rospy.init_node("phidgets_node")
        #rospy.init_node("phidgets_node", log_level=rospy.DEBUG)

        # Call base class initializer, which starts a ros node with a name and log_level
        # It then opens and attaches Phidget device
        self.interfaceKit = InterfaceKit()
        self.encoder = Encoder()
        # self.bridge = Bridge()

        # initialize this to nan to indicate we haven't homed the location
        self.sled_pos = float('nan')  # position of sled in millimeters
        self.encoder_rear_offset = 0
        self.encoder_front_offset = 0

        # Open the devices and wait for them to attach
        self._attachPhidget(self.interfaceKit, "Interface Kit")
        self._attachPhidget(self.encoder, "Encoder board")
        # self._attachPhidget(self.bridge, "Wheatstone Bridge")

        self.params = rospy.get_param("/phidgets")
        self.sled_params = self.params["sled"]

        self.sensor_pub = rospy.Publisher("/actuator_states/raw/", ActuatorStates, queue_size=10)
        self.sensor_processed_pub = rospy.Publisher("/actuator_states/proc", ActuatorStatesProcessed, queue_size=10)
        self.limit_sub = rospy.Subscriber("/pololu/limit_switch", LimitSwitch, self.limit_callback, queue_size=10)
        self.sled_is_homed = False

       # enable both the sled encoders
        self.encoder.setEnabled(self.sled_params["encoder_index"]["left"], True)
        self.encoder.setEnabled(self.sled_params["encoder_index"]["right"], True)

        # Display info of the devices
        self._displayDeviceInfo(self.encoder)
        self.displayInterfaceKitInfo()
        # self.displayBridgeInfo()

    def _attachPhidget(self, phidget, name):
        """Open and wait for the Phidget object to attach"""
        try:
            phidget.openPhidget()
            rospy.loginfo("Waiting for %s to attach....", name)
            phidget.waitForAttach(10000) # 10 s
        except:
            rospy.logerr("%s did not attach!", name)

    def _displayDeviceInfo(self, phidget):
        """Display relevant info about device"""
        rospy.logdebug("Attached: %s", phidget.isAttached())
        rospy.logdebug("Type: %s", phidget.getDeviceName())
        rospy.logdebug("Serial No.: %s", phidget.getSerialNum())
        rospy.logdebug("Version: %s", phidget.getDeviceVersion())

    def displayInterfaceKitInfo(self):
        """Display relevant info about interface kit"""
        self._displayDeviceInfo(self.interfaceKit)
        rospy.logdebug("Number of Digital Inputs: %i", self.interfaceKit.getInputCount())
        rospy.logdebug("Number of Digital Outputs: %i", self.interfaceKit.getOutputCount())
        rospy.logdebug("Number of Sensor Inputs: %i", self.interfaceKit.getSensorCount())
        rospy.logdebug("Min data rate: %i, Max data rate: %i", self.interfaceKit.getDataRateMin(0), self.interfaceKit.getDataRateMax(0))

    def displayBridgeInfo(self):
        """Display relevant info about wheatstone bridge"""
        self._displayDeviceInfo(self.bridge)
        rospy.logdebug("Number of bridge inputs: %i", self.bridge.getInputCount())
        rospy.logdebug("Data Rate Max: %d", self.bridge.getDataRateMax())
        rospy.logdebug("Data Rate Min: %d", self.bridge.getDataRateMin())
        rospy.logdebug("Input Value Max: %d", self.bridge.getBridgeMax(0))
        rospy.logdebug("Input Value Min: %d", self.bridge.getBridgeMin(0))

    def limit_callback(self, limit_switch):
        """Callback for the current state of the switches. If this is the first
        time they have been pressed, set flag to indicate the sled has been homed"""
        if limit_switch.rear and not self.sled_is_homed:
            self.sled_pos = self.sled_params["rear_pos"]
            self.encoder_rear_offset = self.pollEncoders()
            self.sled_is_homed = True
        #elif limit_switch.front:
        #    self.sled_pos = self.sled_params["front_pos"]
        #    self.encoder_front_offset = self.pollEncoders()
        #    self.sled_is_homed = True


    def pollEncoders(self):
        """Return averaged value of sled left and right encoder values"""
        sled_left_ticks = self.encoder.getPosition(self.sled_params["encoder_index"]["left"])
        sled_right_ticks = self.encoder.getPosition(self.sled_params["encoder_index"]["right"])
        average = (-sled_left_ticks + sled_right_ticks) / 2
        rospy.logdebug("left right tick average:  %.2f", average)
        return average


    def pollLinearActuator(self, name):
        """Poll the sensor for its raw value. Then convert that value
        to the actual position in millimeters. Return as a SensorValuePair"""
        device = self.params[name]
        index = device["sensor_index"]
        min_pot_val = device["min"]
        max_pot_val = device["max"]
        physical_length = device["length"]
        # Create objects to fill and later send over message
        raw_val = self.interfaceKit.getSensorValue(index)
        # Convert the potentiometer value to millimeter position
        pos = (raw_val - min_pot_val) * (physical_length / (max_pot_val - min_pot_val))

        return pos

    def sendProcessedMessage(self, actuator_states):
        """Process the interface sensor message to average and get the positions
        of the motors"""
        proc = ActuatorStatesProcessed()
        proc.header = actuator_states.header
        proc.arm = (actuator_states.arm_left + actuator_states.arm_right) / 2
        proc.bucket = (actuator_states.bucket_left + actuator_states.bucket_right) / 2
        proc.sled = actuator_states.sled

        self.sensor_processed_pub.publish(proc)


    def pollSensors(self):
        """Poll and publish all the sensor values as ActuatorStates message"""
        ticks_per_mm = self.sled_params["ticks_per_mm"]

        actuator_states = ActuatorStates() # create object to store message components to send on topic
        header = Header()

        actuator_states.header.stamp = rospy.Time.now()
        actuator_states.arm_left = self.pollLinearActuator("arm_left")
        actuator_states.arm_right = self.pollLinearActuator("arm_right")
        actuator_states.bucket_left = self.pollLinearActuator("bucket_left")
        actuator_states.bucket_right = self.pollLinearActuator("bucket_right")

        sled_left_ticks = self.encoder.getPosition(self.sled_params["encoder_index"]["left"])
        sled_right_ticks = self.encoder.getPosition(self.sled_params["encoder_index"]["right"])

        rospy.logdebug("left ticks = %d", sled_left_ticks)
        rospy.logdebug("right ticks = %d", sled_right_ticks)

        if self.sled_is_homed:
            rospy.logdebug("left pos = %.2f", sled_left_ticks / ticks_per_mm)
            rospy.logdebug("right pos = %.2f", sled_right_ticks / ticks_per_mm)
            sled_pos = (self.pollEncoders() - self.encoder_rear_offset) / ticks_per_mm
        else:
            sled_pos = float('nan') # set this so everybody knows we haven't homed the sled yet

        actuator_states.sled = sled_pos
        # TODO: add sled_pos to position when that is setup

        # actuator_states.sled = self.interfaceKit.getSensorValue(self.params["sled"]["sensor_index"])
        self.sensor_pub.publish(actuator_states)
        self.sendProcessedMessage(actuator_states)


    def run(self):
        """Run the main ros loop"""
        rospy.loginfo("Starting phidgets node loop")
        r_time = rospy.Rate(10) #10 Hz looping
        while not rospy.is_shutdown():
            self.pollSensors()
            r_time.sleep()


if __name__ == "__main__":
    try:
        node = PhidgetNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting phidgets node")
