#!/usr/bin/env python3


import rospy
import sys

from math import atan2, tanh, sqrt, copysign, pi
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import actionlib
import tf

from lab2.msg import NavTargetAction, NavTargetResult, NavTargetFeedback


class Driver:
	def __init__(self, position_source, threshold=0.1):
		# Goal will be set later. The action server will set the goal; you don't set it directly
		self.goal = None
		self.threshold = threshold

		self.avoiding = False
		self.avoid_direction = None

		self.transform_listener = tf.TransformListener()

		# Publisher before subscriber
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.target_pub = rospy.Publisher('current_target', Marker, queue_size=1)

		# Subscriber after publisher
		self.sub = rospy.Subscriber('base_scan', LaserScan, self._callback, queue_size=1)

		# Action client
		self.action_server = actionlib.SimpleActionServer('nav_target', NavTargetAction, execute_cb=self._action_callback, auto_start=False)
		self.action_server.start()

	@classmethod
	def zero_twist(cls):
		"""This is a helper class method to create and zero-out a twist"""
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0

		return command

	# Respond to the action request.
	def _action_callback(self, goal):
		print('_action_callback')
		""" This gets called when an action is received by the action server
		@goal - this is the new goal """
		rospy.loginfo(f'Got an action request for ({goal.goal.point.x:.2f}, {goal.goal.point.y:.2f})')

		# Set the goal.
		self.goal = goal.goal

		# Build a marker for the goal point
		#   - this prints out the green dot in RViz (the current goal)
		marker = Marker()
		marker.header.frame_id = goal.goal.header.frame_id
		marker.header.stamp = rospy.Time.now()
		marker.id = 0
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position = goal.goal.point
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0		
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		# Wait until we're at the goal.  Once we get there, the callback that drives the robot will set self.goal
		# to None.
		while self.goal:
			self.target_pub.publish(marker)
			rospy.sleep(0.1)

		rospy.loginfo('Action completed')

		# Build a result to send back
		result = NavTargetResult()
		result.success.data = True

		self.action_server.set_succeeded(result)

		# Get rid of the marker
		marker.action = Marker.DELETE
		self.target_pub.publish(marker)

	def _callback(self, lidar):
		# If we have a goal, then act on it, otherwise stay still
		if self.goal:
			# Update the timestamp on the goal and figure out where it it now in the base_link frame.
			self.goal.header.stamp = rospy.Time.now() - rospy.Time(0.1)
			target = self.transform_listener.transformPoint('base_link', self.goal)

			# rospy.loginfo(f'Target: ({target.point.x:.2f}, {target.point.y:.2f})')

			# Are we close enough?  If so, then remove the goal and stop
			distance = sqrt(target.point.x ** 2 + target.point.y ** 2)

			feedback = NavTargetFeedback()
			feedback.distance.data = distance
			self.action_server.publish_feedback(feedback)

			if distance < self.threshold:
				self.goal = None
				command = Driver.zero_twist()
			else:
				command = self.get_twist((target.point.x, target.point.y), lidar)
		else:
			command = Driver.zero_twist()

		self.cmd_pub.publish(command)

	# This is the function that controls the robot.
	#
	# Inputs:
	# 	target:	a tuple with the (x, y) coordinates of the target point, in the robot's coordinate frame (base_link).
	# 			x-axis is forward, y-axis is to the left.
	# 	lidar:	a LaserScan message with the current data from the LiDAR.  Use this for obstacle avoidance.
	#           This is the same as your go and stop code
	def get_twist(self, target, lidar):
		command = Driver.zero_twist()

		# TODO:
		#  Step 1) Calculate the angle the robot has to turn to in order to point at the target
		#  Step 2) Set your speed based on how far away you are from the target, as before
		#  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it

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

		# Find indecies less than 19cm (+50%)
		x_infront = x <= 0.19 * 1.5

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

		# If less than 1m in front of a wall, avoid
		if nearest_obstacle < 0.75 and nearest_obstacle < distance and not self.avoiding:
			self.avoiding = True
			self.avoid_direction = -np.sign(nearest_obstacle_180_angle)

		if nearest_obstacle_180 > 1 and self.avoiding:
			self.avoiding = False
			self.avoid_direction = None

		if self.avoiding:
			rospy.loginfo(f'Avoiding! In front: {nearest_obstacle:.4f}m ({nearest_obstacle_angle:.2f}*), 180: {nearest_obstacle_180:.4f}m ({nearest_obstacle_180_angle:.2f}*)')

			x_speed = 1 / (1 + np.exp(5 - nearest_obstacle_180 * 5)) if nearest_obstacle > 0.38 else 0
			angular_speed = 0.15 * self.avoid_direction if nearest_obstacle < 1 else 0

		# This sets the move forward speed (as before)
		command.linear.x = x_speed
		# This sets the angular turn speed (in radians per second)
		command.angular.z = angular_speed

		return command

if __name__ == '__main__':
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver('odom', threshold=0.1)

	rospy.spin()
