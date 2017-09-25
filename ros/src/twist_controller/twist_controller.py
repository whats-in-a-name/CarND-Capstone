from yaw_controller import YawController
from pid import PID

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        vehicle_mass = kwargs['vehicle_mass']
        fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        decel_limit = kwargs['decel_limit']
        accel_limit = kwargs['accel_limit']
        wheel_radius = kwargs['wheel_radius']
        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        min_speed = kwargs['min_speed']

        linear_p_term = kwargs['linear_p_term']
        linear_i_term = kwargs['linear_i_term']
        linear_d_term = kwargs['linear_d_term']

        # Calculate required braking torque according to vehicle dynamics?
        _total_vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        # Use F = ma to calculate the
        # F_max = m * a_max
        # T_max = F_max * r = m * r * a_max
        # Assume all CoFs (Coefficient of Frictions) are 1
        self._brake_torque_base = _total_vehicle_mass * wheel_radius
        self.yaw_controller = YawController(wheel_base, steer_ratio,
                                            min_speed, max_lat_accel, max_steer_angle)

        # Tune the parameters in dbw_node
        self.linear_pid = PID(linear_p_term, linear_i_term, linear_d_term,
                              decel_limit, accel_limit)

        self._now = None
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
