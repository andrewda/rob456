#!/usr/bin/env python3

import sys
import rospy

from new_driver import Driver
from obstacle_avoidance import ObstacleAvoidance

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

		self.oa = ObstacleAvoidance()


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

		# Calculate x_speed and angular speed from obstacle avoidance function
		x_speed, angular_speed = self.oa.obstacle_avoidance(target, lidar_scan, thetas)

		command.linear.x = x_speed
		command.angular.z = angular_speed

		return command


if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
