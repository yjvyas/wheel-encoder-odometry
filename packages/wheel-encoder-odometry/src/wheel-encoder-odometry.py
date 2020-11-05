#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
from math import pi

class WheelEncoderOdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(WheelEncoderOdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Integrated distance
        self.encoder_ticks = [0, 0]
        self.distance_wheels = [0, 0]
        self.velocity_wheels = [0, 0]

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', \
            WheelEncoderStamped, self.cb_encoder_data, callback_args=[0])
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', \
            WheelEncoderStamped, self.cb_encoder_data, callback_args=[1])
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/wheel_distance/left', Float32, queue_size=30)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/wheel_distance/right', Float32, queue_size=30)

        self.log("Initialized")

    def cb_encoder_data(self, msg, args):

        index_wheel = args[0]
        ticks_difference = msg.data - self.encoder_ticks[index_wheel]

        if self.velocity_wheels[index_wheel] >= 0:
            self.distance_wheels[index_wheel] += 2*pi*self._radius*ticks_difference/msg.resolution
        else:
            self.distance_wheels[index_wheel] -= 2*pi*self._radius*ticks_difference/msg.resolution

        self.encoder_ticks[index_wheel] = msg.data
    
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(30) # 30Hz
        while not rospy.is_shutdown():
            self.pub_integrated_distance_left.publish(self.distance_wheels[0])
            self.pub_integrated_distance_right.publish(self.distance_wheels[1])
            rate.sleep()

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        self.velocity_wheels = [msg.vel_left, msg.vel_right]

if __name__ == '__main__':
    node = WheelEncoderOdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    node.run()
    
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")