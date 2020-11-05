#!/usr/bin/env python3
import numpy as np
import os
import rospy
import sys
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
from math import pi

class WheelRadiusEstimation(DTROS):

    def __init__(self, node_name, distance):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(WheelRadiusEstimation, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self.distance = float(distance)
        self.resolution = 135
        rospy.loginfo('Distance set is {}m.'.format(distance))

        # get dynamic params
        self.radius_left = 0
        self.radius_right = 0

        # Integrated distance
        self.encoder_ticks = [None, None]
        self.distance_wheels = [0, 0]
        self.velocity_wheels = [0, 0]

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', \
            WheelEncoderStamped, self.cb_encoder_data, callback_args=[0])
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', \
            WheelEncoderStamped, self.cb_encoder_data, callback_args=[1])

        self.log("Initialized wheel radius estimation. When this starts ONLY execute a forward or backward direction command!")

    def cb_encoder_data(self, msg, args):

        index_wheel = args[0]
        if self.encoder_ticks[index_wheel] is None:
            self.encoder_ticks[index_wheel] = msg.data
        else:
            self.encoder_ticks[index_wheel] += msg.data - self.encoder_ticks[index_wheel]
    
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(10) # 30Hz

        while not rospy.is_shutdown():
            if self.encoder_ticks[0] is not None and self.encoder_ticks[1] is not None:
                self.radius_left = self.distance*self.resolution/(2*pi*self.encoder_ticks[0])
                self.radius_right = self.distance*self.resolution/(2*pi*self.encoder_ticks[1])
            rospy.loginfo('estimated radius left wheel: {}'.format(self.radius_left))
            rospy.loginfo('estimated radius right wheel: {}'.format(self.radius_right))
            rate.sleep()

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    node = WheelRadiusEstimation(node_name='wheel_radius_estimation', distance=myargv[1])
    # Keep it spinning to keep the node alive
    node.run()
    
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")