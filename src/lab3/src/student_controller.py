#!/usr/bin/env python3


import sys
import rospy
import signal

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

		self._last_plan = None


	def _update_path_callback(self, _a):
		rospy.loginfo(f'Got _update_path_callback: {_a}')

		self.goal = None

		self.do_map_update(self._point, self._map, self._map_data)

		rospy.loginfo('Done _update_path_callback!')


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
		# TODO: implement this
		# rospy.loginfo(f'Distance: {distance}')
		pass


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


	def do_map_update(self, point, map, map_data):
		if point is None:
			rospy.loginfo('Point is none, returning')
			return

		if self._last_plan is not None and self._last_plan - rospy.Time.now() > rospy.Time(30):
			rospy.loginfo('Got path update request, but already updated too recently')
			return

		if len(self._waypoints) > 0:
			rospy.loginfo(f'Got path update request, but {len(self._waypoints)} waypoints remain')
			return


		rospy.loginfo('Updating path.')
		rospy.loginfo(self.action_client.get_state())

		im = np.array(map.data).reshape(4000, 4000)
		im_thresh = path_planning.convert_image(im, 0.7, 0.9)

		im_thresh_fattened = path_planning.fatten_image(im_thresh, 4)

		x = int(point.point.x / map_data.resolution + 2000)
		y = int(point.point.y / map_data.resolution + 2000)

		robot_start_loc = (x, y)

		if self.goal is None:
			all_unseen = exploring.find_all_possible_goals(im_thresh_fattened)
			self.goal = exploring.find_best_point(im_thresh_fattened, all_unseen, robot_loc=robot_start_loc)

		# plot_with_explore_points(im_thresh_fattened, zoom=0.1, robot_loc=robot_start_loc, best_pt=best_unseen)

		rospy.loginfo(f'Got best unseen! From {robot_start_loc} to {self.goal}')

		path = path_planning.dijkstra(im_thresh_fattened, robot_start_loc, self.goal)
		waypoints = exploring.find_waypoints(im_thresh, path)
		# path_planning.plot_with_path(im, im_thresh_fattened, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

		waypoints = [((x - 2000) * map_data.resolution, (y - 2000) * map_data.resolution) for x, y in waypoints]

		rospy.loginfo(waypoints)

		controller.set_waypoints(waypoints)
		# controller.send_points()

		rospy.loginfo('Points have been sent!')


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()

	# This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	controller.set_waypoints(((-4, -3), (-4, 0), (5, 0)))

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()
