import rospy

from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        vehicle_mass = kwargs['vehicle_mass']
        fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
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
                              self.decel_limit, accel_limit)

        self._now = None

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
        if _control_correction > 0:
            # Throttle scale
            accel = _control_correction * 1
            # Should multiple it by the nominal value of control input
            throttle = accel
        else:
            # When using brake torque message with DBW
            # Factor to achieve around 20000 max brake torque?
            # Now max brake torque is around 1800
            decel = abs(_control_correction)
            if decel > self.brake_deadband:
                brake = decel/float(self.decel_limit)

        # Steer and steer ratio
        steering = self.yaw_controller.get_steering(linear_velocity_setpoint,
                                                    angular_velocity_setpoint, current_linear_velocity)
        return throttle, brake, steering
