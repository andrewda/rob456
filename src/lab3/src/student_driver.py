#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver

from math import atan2, tanh, sqrt, copysign, pi
import numpy as np


class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self):
		super().__init__('odom', threshold=0.3)

		self.avoiding = False
		self.avoid_direction = None

	def get_twist(self, target, lidar):
		'''
		This function is called whenever there a current target is set and there is a lidar data
		available.  This is where you should put your code for moving the robot.  The target point
		is in the robot's coordinate frame.  The x-axis is positive-forwards, and the y-axis is
		positive to the left.

		The example sets constant velocities, which is clearly the wrong thing to do.  Replace this
		code with something that moves the robot more intelligently.

		Parameters:
			target:		The current target point, in the coordinate frame of the robot (base_link) as
						an (x, y) tuple.
			lidar:		A LaserScan containing the new lidar data.

		Returns:
			A Twist message, containing the commanded robot velocities.
		'''
		# angle = atan2(target[1], target[0])
		# distance = sqrt(target[0] ** 2 + target[1] **2)
		# rospy.loginfo(f'Distance: {distance:.2f}, angle: {angle:.2f}')

		# # This builds a Twist message with all elements set to zero.
		# command = Driver.zero_twist()

		# # Forwards velocity goes here, in meters per second.
		# command.linear.x = 0.1

		# # Rotational velocity goes here, in radians per second.  Positive is counter-clockwise.
		# command.angular.z = 0.1

		# return command

		command = Driver.zero_twist()

		angle_min = lidar.angle_min
		angle_max = lidar.angle_max
		num_readings = len(lidar.ranges)

		lidar_scan = np.array(lidar.ranges)

		thetas = np.linspace(angle_min, angle_max, num_readings)

		angle = atan2(target[1], target[0]) * 180/pi
		distance = sqrt(target[0] ** 2 + target[1] ** 2)

		rospy.loginfo(f'To target: {angle}*, {distance}m')

		# Get 1/2 width values using trig
		x = np.abs(lidar.ranges * np.sin(thetas))

		# Find indecies less than 19cm (+25%)
		x_infront = x <= 0.19 * 1.25

		# Get thetas and scan distances
		thetas_infront = thetas[x_infront]
		scans_infront = lidar_scan[x_infront]

		# Find closest index
		idx_closest = np.argmin(scans_infront)
		idx_closest_180 = np.argmin(lidar_scan)

		rospy.loginfo('000')

		# Get shortest scan distance
		nearest_obstacle = scans_infront[idx_closest]
		nearest_obstacle_180 = lidar_scan[idx_closest_180]

		nearest_obstacle_angle = thetas_infront[idx_closest] * 180/pi
		nearest_obstacle_180_angle = thetas[idx_closest_180] * 180/pi

		shortest = min(distance, nearest_obstacle)

		# Adjust speeds along sigmoid function
		x_speed = 8 / (1 + np.exp(5 - shortest)) if np.abs(angle) < 30 else 0
		angular_speed = (2 / (1 + np.exp(-angle / 10)) - 1) * 0.5

		rospy.loginfo('100')

		######################
		# Obstacle Avoidance #
		######################

		# If less than 1m in front of a wall, avoid
		if nearest_obstacle < 0.5 and nearest_obstacle < distance and not self.avoiding:
			self.avoiding = True
			self.avoid_direction = -np.sign(nearest_obstacle_180_angle)

		if nearest_obstacle_180 > 0.75 and self.avoiding:
			self.avoiding = False
			self.avoid_direction = None

		if self.avoiding:
			rospy.loginfo(f'Avoiding! In front: {nearest_obstacle:.4f}m ({nearest_obstacle_angle:.2f}*), 180: {nearest_obstacle_180:.4f}m ({nearest_obstacle_180_angle:.2f}*)')

			x_speed = 1 / (1 + np.exp(5 - nearest_obstacle_180 * 5)) if nearest_obstacle > 0.38 else 0
			angular_speed = 0.15 * self.avoid_direction if nearest_obstacle < 0.5 else 0

		rospy.loginfo('200')

		# This sets the move forward speed (as before)
		command.linear.x = x_speed
		# This sets the angular turn speed (in radians per second)
		command.angular.z = angular_speed

		rospy.loginfo(command)

		return command


if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
