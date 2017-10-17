from  yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
	def __init__(self, *args, **kwargs):
		# TODO: Implement
		self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
											kwargs['min_speed'] + ONE_MPH, kwargs['max_lat_accel'],
											kwargs['max_steer_angle'])
		#self.throttle_pid = PID(kp=0.05, ki=0.015, kd=0.15, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
		self.throttle_pid = PID(kp=0.5, ki=0.5, kd=0.5, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
		self.s_lpf = LowPassFilter(tau = 3, ts = 1)
		self.min_speed = kwargs['min_speed']
		self.wheel_base = kwargs['wheel_base']
		self.total_mass = kwargs['total_mass']
		
		self.prev_time = None

	def control(self, *args, **kwargs):
		# TODO: Change the arg, kwarg list to suit your needs
		# Return throttle, brake, steer
		target_velocity_linear_x = args[0]
		target_velocity_angular_z = args[1]
		current_velocity_linear_x = args[2]
		current_velocity_angular_z = args[3]
		dbw_enabled = args[4]

		throttle = 0.0
		brake = 0.0

		if not dbw_enabled: 
			self.throttle.reset()
			return 0, 0, 0

		# Compute difference between target and current velocity as CTE for throttle. 
		diff_velocity = target_velocity_linear_x - current_velocity_linear_x

		current_time = rospy.get_time()
		dt = 0
		if self.prev_time is not None: 
			dt = current_time - self.prev_time
		self.prev_time = current_time
		
		velocity_controller = 0
		if dt > 0:
			velocity_controller = self.throttle_pid.step(diff_velocity, dt)

		if velocity_controller > 0:
			throttle = velocity_controller
			if throttle > 0.1 :
				throttle = 0.3 
		elif velocity_controller < 0:
			brake = abs( self.total_mass * velocity_controller * self.wheel_base)



		steering = self.yaw_controller.get_steering(target_velocity_linear_x, target_velocity_angular_z, current_velocity_linear_x)
		steering = self.s_lpf.filt(steering)

		return throttle, brake, steering
