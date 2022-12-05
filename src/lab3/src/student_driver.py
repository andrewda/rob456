#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver
import obstacle_avoidance

from math import atan2, tanh, sqrt, copysign, pi
import numpy as np

from std_msgs.msg import String

class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self):
		super().__init__('odom', threshold=0.5)

		self.avoiding = False
		self.avoid_direction = None

		self.goal_direction = None

		self.ao = obstacle_avoidance.ObstacleAvoidance()


	def get_twist(self, target, lidar):
		'''
		Parameters:
			target:		The current target point, in the coordinate frame of the robot (base_link) as
						an (x, y) tuple.
			lidar:		A LaserScan containing the new lidar data.

		Returns:
			A Twist message, containing the commanded robot velocities.
		'''
		command = Driver.zero_twist()

		angle_min = lidar.angle_min
		angle_max = lidar.angle_max
		num_readings = len(lidar.ranges)

		lidar_scan = np.array(lidar.ranges)
		thetas = np.linspace(angle_min, angle_max, num_readings)

		try:
			x_speed, angular_speed = self.ao.obstacle_avoidance(target, lidar_scan, thetas)

			rospy.loginfo(f'x_speed = {x_speed}, angular_speed = {angular_speed}')

			command.linear.x = x_speed
			command.angular.z = angular_speed

			return command
		except Exception as e:
			rospy.loginfo(e)
			raise e

		angle = atan2(target[1], target[0]) * 180/pi
		distance = sqrt(target[0] ** 2 + target[1] ** 2)

		# Get 1/2 width values using trig
		x = np.abs(lidar.ranges * np.sin(thetas))

		# Find indecies less than half the robot width (19cm) +25% buffer
		x_infront = x <= 0.19 * 1.25

		# Get thetas and scan distances
		thetas_infront = thetas[x_infront]
		scans_infront = lidar_scan[x_infront]

		# Find closest index
		idx_closest = np.argmin(scans_infront)
		idx_closest_180 = np.argmin(lidar_scan)

		# Get shortest scan distance
		nearest_obstacle = scans_infront[idx_closest]
		nearest_obstacle_180 = lidar_scan[idx_closest_180]

		nearest_obstacle_angle = thetas_infront[idx_closest] * 180/pi
		nearest_obstacle_180_angle = thetas[idx_closest_180] * 180/pi

		shortest = min(distance, nearest_obstacle)

		# Adjust speeds along sigmoid function
		x_speed = 8 / (1 + np.exp(5 - shortest)) if np.abs(angle) < 30 else 0
		angular_speed = (2 / (1 + np.exp(-angle / 10)) - 1) * 0.5

		######################
		# Obstacle Avoidance #
		######################

		idx_goal = np.argmin(np.abs(thetas * 180/pi - angle))
		goal_scan = lidar_scan[idx_goal]

		# If less than 1m in front of a wall, avoid
		if nearest_obstacle < 0.5 and nearest_obstacle < distance and not self.avoiding:
			self.avoiding = True
			self.avoid_direction = -np.sign(nearest_obstacle_180_angle)
			self.goal_direction = np.sign(angle)

		if self.avoiding and (nearest_obstacle_180 > 0.75 or self.goal_direction != np.sign(angle)):
			self.avoiding = False
			self.avoid_direction = None
			self.goal_direction = None

		if self.avoiding:
			x_speed = 2 / (1 + np.exp(5 - nearest_obstacle_180 * 5)) if nearest_obstacle > 0.38 else 0
			angular_speed = 0.15 * self.avoid_direction if nearest_obstacle < 0.5 else 0

		command.linear.x = x_speed
		command.angular.z = angular_speed

		rospy.loginfo(x_speed)

		if x_speed < 0.001:
			rospy.loginfo('Updating path!')
			self._update_path_pub.publish('abc')

		return command


if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
