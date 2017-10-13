import rospy
import math

from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        vehicle_mass = kwargs['vehicle_mass']
        fuel_capacity = kwargs['fuel_capacity']
        decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
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
                              decel_limit, self.accel_limit)

        self._now = None
        self.v_min = min_speed

    def reset(self):
        """
        Reset PID when dbw_enable event is disabled
        :return:
        """
        self.linear_pid.reset()
        self._now = None

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        linear_velocity_setpoint = kwargs['linear_velocity_setpoint']
        angular_velocity_setpoint = kwargs['angular_velocity_setpoint']
        current_linear_velocity = kwargs['current_linear_velocity']

        # Sample time interval:
        timestamp = rospy.get_time()
        if not self._now:
            _sample_time = 0.02  # 50 Hz
        else:
            _sample_time = timestamp - self._now
        self._now = timestamp

        _error = linear_velocity_setpoint - current_linear_velocity

        _control_correction = self.linear_pid.step(_error, _sample_time)

        throttle = 0
        brake = 0
        if linear_velocity_setpoint == 0.0 and current_linear_velocity < self.v_min:
            brake = self._brake_torque_base
        # 3m/s
        elif linear_velocity_setpoint > 0 and _control_correction > 0.0 and current_linear_velocity < 3:
            throttle = self.launch_control(current_linear_velocity)
        elif _control_correction >= 0:
            throttle = _control_correction
        else:
            brake = -1.0 * self._brake_torque_base * _control_correction

        # Steer and steer ratio
        steering = self.yaw_controller.get_steering(linear_velocity_setpoint,
                                                    angular_velocity_setpoint, current_linear_velocity)

        return throttle, brake, steering

    def launch_control(self, vel):
        return self._logistic(self.accel_limit, vel)

    def _logistic(self, max_x, x):
        """
        _steepness is determined empirically
        :param max_x:
        :param x:
        :return:
        """
        _steepness = 0.3
        _mid_point = 4
        return max_x / (1 + pow(math.exp(1), _steepness * (_mid_point - x)))
