#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver

from math import atan2, tanh, sqrt, copysign, pi
import numpy as np

from std_msgs.msg import String

class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self):
		super().__init__('odom', threshold=0.3)

		self.avoiding = False
		self.avoid_direction = None

		self.do_180 = False
		self.angle_180 = None


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

		command = Driver.zero_twist()

		angle_min = lidar.angle_min
		angle_max = lidar.angle_max
		num_readings = len(lidar.ranges)

		lidar_scan = np.array(lidar.ranges)

		thetas = np.linspace(angle_min, angle_max, num_readings)
		# rospy.loginfo(f'LINSPACE: {angle_min}, {angle_max}')

		angle = atan2(target[1], target[0]) * 180/pi
		distance = sqrt(target[0] ** 2 + target[1] ** 2)

		# rospy.loginfo(f'To target: {angle}*, {distance}m')

		# Get 1/2 width values using trig
		x = np.abs(lidar.ranges * np.sin(thetas))

		# Find indecies less than 19cm (+25%)
		x_infront = x <= 0.19 #* 1.25

		# Get thetas and scan distances
		thetas_infront = thetas[x_infront]
		scans_infront = lidar_scan[x_infront]

		# Find closest index
		idx_closest = np.argmin(scans_infront)
		idx_closest_180 = np.argmin(lidar_scan)

		# half_idx = int(len(lidar_scan) / 2)
		# scan_left = lidar_scan[:half_idx]
		# scan_right = lidar_scan[half_idx:]

		# avg_scan_left = np.mean(scan_left)
		# avg_scan_right = np.mean(scan_right)

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

		if not self.avoiding and nearest_obstacle_180 < 0.88:
			self.avoiding = True

		if self.avoiding and (nearest_obstacle_180 > 0.88 or goal_scan > distance or goal_scan > 4): # TODO: and we see the waypoint...
			self.avoiding = False

		if self.avoiding:
			furthest_idx = np.argmax(lidar_scan[lidar_scan < 0.88])
			furthest_angle = thetas[furthest_idx] * 180/pi

			rospy.loginfo(f'Avoiding, furthest angle: {furthest_angle}')
			angular_speed = 0.15 * np.sign(furthest_angle)

			if self.do_180:
				if abs(angle - self.angle_180) < 5:
					self.do_180 = False
					self.angle_180 = None
				else:
					x_speed = 0
					angular_speed = 0.15

			if nearest_obstacle < 0.44:

				self.do_180 = True
				self.angle_180 = (angle + 180) % 360

				x_speed = 0
				angular_speed = 0.15


		# if nearest_obstacle < 0.44 and not self.avoiding:
		# 	self.avoiding 


		# This sets the move forward speed (as before)
		command.linear.x = x_speed
		# This sets the angular turn speed (in radians per second)
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
