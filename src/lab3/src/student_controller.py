#!/usr/bin/env python3

import sys
import rospy
import signal
import time

from controller import RobotController

import path_planning as path_planning
import exploring as exploring
import numpy as np

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String


class StudentController(RobotController):
	'''
	This class allows you to set waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()

		self.goal = None

		self._update_path_sub = rospy.Subscriber('update_path', String, self._update_path_callback, queue_size=10)

		self.last_plan_at = None

		self.last_progress_at = None
		self.last_distance = None
		self.last_waypoint_count = None


	def _update_path_callback(self, _a=''):
		self.goal = None
		self.do_path_update(self._point, self._map, self._map_data)


	def distance_update(self, distance):
		'''
		This function is called every time the robot moves towards a goal.  If you want to make sure that
		the robot is making progress towards it's goal, then you might want to check that the distance to
		the goal is generally going down.  If you want to change where the robot is heading to, you can
		make a call to set_waypoints here.  This call will override the current set of waypoints, and the
		robot will start to drive towards the first waypoint in the new list.

		Parameters:
			distance:	The distance to the current goal.
		'''

		# Check if we've progressed
		if self.last_progress_at is None or self.last_waypoint_count != len(self._waypoints) or (self.last_waypoint_count == len(self._waypoints) and self.last_distance > (distance + 0.05)):
			self.last_progress_at = time.time()
			self.last_distance = distance
			self.last_waypoint_count = len(self._waypoints)

		# If we haven't progressed in 10 seconds, replan
		time_since_update = time.time() - self.last_progress_at
		if time_since_update > 10:
			self.do_path_update(self._point, self._map, self._map_data, force=True)

		rospy.loginfo(f'Distance to goal: {distance} (# waypoints: {len(self._waypoints)}, time since progress: {time_since_update:.2f})')


	def map_update(self, point, map, map_data):
		'''
		This function is called every time a new map update is available from the SLAM system.  If you want
		to change where the robot is driving, you can do it in this function.  If you generate a path for
		the robot to follow, you can pass it to the driver code using set_waypoints().  Again, this will
		override any current set of waypoints that you might have previously sent.

		Parameters:
			point:		A PointStamped containing the position of the robot, in the map coordinate frame.
			map:		An OccupancyGrid containing the current version of the map.
			map_data:	A MapMetaData containing the current map meta data.
		'''
		pass


	def do_path_update(self, point, map, map_data, force=False):
		# If we don't have our current location, return
		if point is None or map is None:
			rospy.loginfo('Point is none, skipping path update')
			return

		# If we already made a path plan recently, return
		if self.last_plan_at is not None and (time.time() - self.last_plan_at) < 5:
			rospy.loginfo(f'Got path update request, but already updated too recently {time.time() - self.last_plan_at}')
			return

		# If we still have waypoints left to go, return (unless we force the generation)
		if not force:
			if self._waypoints is not None and len(self._waypoints) > 0:
				rospy.loginfo(f'Got path update request, but {len(self._waypoints)} waypoints remain')
				return

		self.last_plan_at = time.time() 

		rospy.loginfo(f'Updating path (force={force})')

		# Reshape the 1D map data to an image grid and threshold it
		im = np.array(map.data).reshape(map.info.height, map.info.width)
		im_thresh = path_planning.convert_image(im, 0.7, 0.9)

		# Fatten the walls so the robot has enough room to navigate our path
		fatten_pixels = int(np.ceil(0.19 / map_data.resolution)) + 1
		im_thresh_fattened = path_planning.fatten_image(im_thresh, fatten_pixels)

		# Get the robot's starting position in map coordinates
		x = int(point.point.x / map_data.resolution + map.info.width / 2)
		y = int(point.point.y / map_data.resolution + map.info.height / 2)
		robot_start_loc = (x, y)

		# If we don't already have a goal, find one from the list of all possible goals
		if self.goal is None:
			all_unseen = exploring.find_all_possible_goals(im_thresh_fattened)
			if all_unseen is None:
				rospy.loginfo('Done, Stopped!')
				return

			self.goal = exploring.find_best_point(im_thresh_fattened, all_unseen, robot_loc=robot_start_loc)

		rospy.loginfo(f'Got best unseen! From {robot_start_loc} to {self.goal}')

		# Plot a path with Dijkstra, and gather waypoints for the robot to follow
		path = path_planning.dijkstra(im_thresh_fattened, robot_start_loc, self.goal)
		waypoints = exploring.find_waypoints(im_thresh, path)

		# Convert waypoints back from map coordinates to robot world coordinates
		waypoints = [((x - 2000) * map_data.resolution, (y - 2000) * map_data.resolution) for x, y in waypoints]

		# Set waypoints in the controller
		controller.set_waypoints(waypoints)

		rospy.loginfo('Points have been sent!')


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()

	# Plan an initial path
	controller._update_path_callback()

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()
