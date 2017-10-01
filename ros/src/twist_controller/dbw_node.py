#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Control scheme
        # Angle control is done by pure pursuit through Autoware
        # Linear control will be done through linear PID in twister_controller
        self.controller = Controller(
            vehicle_mass=rospy.get_param('~vehicle_mass', 1736.35),
            fuel_capacity=rospy.get_param('~fuel_capacity', 13.5),
            brake_deadband=rospy.get_param('~brake_deadband', .1),
            decel_limit=rospy.get_param('~decel_limit', -5),
            accel_limit=rospy.get_param('~accel_limit', 1.),
            wheel_radius=rospy.get_param('~wheel_radius', 0.2413),
            wheel_base=rospy.get_param('~wheel_base', 2.8498),
            steer_ratio=rospy.get_param('~steer_ratio', 14.8),
            max_lat_accel=rospy.get_param('~max_lat_accel', 3.),
            max_steer_angle=rospy.get_param('~max_steer_angle', 8.),
            min_speed=rospy.get_param('~min_speed', 0.0),
            linear_p_term=rospy.get_param('~linear_p_term', 1),
            linear_i_term=rospy.get_param('~linear_i_term', 0.0005),
            linear_d_term=rospy.get_param('~linear_d_term', 0.05),
            lpf_tau=rospy.get_param('~lpf_tau', 0.5),
            lpf_ts=rospy.get_param('~lpf_ts', 1)
        )

        # Target velocities
        self.angular_velocity = 0.0
        self.linear_velocity = 0.0

        # DBW activation state
        self.dbw_enabled = False

        # m/s
        self.LOW_SPEED_THRESHOLD = rospy.get_param('~low_speed_threshold', 3)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        self.subscribers = [
            rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb),
            rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb),
            rospy.Subscriber('/twist_cmd', TwistStamped, self.dbw_twist_cb)
        ]

    def spin(self):
        if rospy.get_param('~use_dbw', True):
            rospy.spin()

    def publish(self, throttle, brake, steer):
        # The numerical issue is taken care by brake_deadband
        if brake > 0:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_PERCENT
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)
        else:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        if not self.dbw_enabled:
            self.controller.reset()
            return

        _linear_velocity = msg.twist.linear.x
        throttle, brake, steer = self.controller.control(
            angular_velocity_setpoint=self.angular_velocity,
            linear_velocity_setpoint=self.linear_velocity,
            current_linear_velocity=_linear_velocity
        )

        # Using launch control to smooth low speed throttle increase
        # e.g. restart after stopping for red light
        if throttle > 0.08 and _linear_velocity < self.LOW_SPEED_THRESHOLD:
            throttle = self.controller.launch_control(_linear_velocity)

        self.publish(throttle=throttle, brake=brake, steer=steer)

    def dbw_twist_cb(self, msg):
        self.angular_velocity = msg.twist.angular.z
        self.linear_velocity = msg.twist.linear.x


if __name__ == '__main__':
    node = DBWNode()
    node.spin()
