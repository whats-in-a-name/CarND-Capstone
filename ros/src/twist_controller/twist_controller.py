from yaw_controller import YawController
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # PID controller(s)
        # _kp_steer = args[0]
        # _ki_steer = args[1]
        # _kd_steer = args[2]

        # pid_steer = PID(_kp_steer, _ki_steer, _kd_steer)
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]
        self.min_speed = args[10]
        self.yawController = YawController(self.wheel_base,
                                           self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        linear_velocity = args[1]
        angular_velocity = args[2]
        current_velocity = args[3]

        # Bad operand type for abs() Vector 3
        # Todo calculate part of the velocity for all three speed

        rospy.loginfo(linear_velocity)

        # Steer and steer ratio
        steer = self.yawController.get_steering(linear_velocity, angular_velocity, current_velocity);
        return 1., 0., 0.
