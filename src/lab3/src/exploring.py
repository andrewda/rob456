#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import path_planning as path_planning
# Our priority queue
import heapq

# Using imageio to read in the image
import imageio

import math

import rospy


# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
	"""Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
	@param im - the image of the SLAM map
	@param im_threshhold - the image of the SLAM map
	@param robot_loc - the location of the robot in pixel coordinates
	@param best_pt - The best explore point (tuple, i,j)
	@param explore_points - the proposed places to explore, as a list"""

	# Putting this in here to avoid messing up ROS
	import matplotlib.pyplot as plt

	fig, axs = plt.subplots(1, 2)
	axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
	axs[0].set_title("original image")
	axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
	axs[1].set_title("threshold image")
	"""
	# Used to double check that the is_xxx routines work correctly
	for i in range(0, im_threshhold.shape[1]-1, 10):
		for j in range(0, im_threshhold.shape[0]-1, 2):
			if is_reachable(im_thresh, (i, j)):
				axs[1].plot(i, j, '.b')
	"""

	# Show original and thresholded image
	if explore_points is not None:
		for p in explore_points:
			axs[1].plot(p[0], p[1], '.b', markersize=2)

	for i in range(0, 2):
		if robot_loc is not None:
			axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
		if best_pt is not None:
			axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
		axs[i].axis('equal')

	for i in range(0, 2):
		# Implements a zoom - set zoom to 1.0 if no zoom
		width = im_threshhold.shape[1]
		height = im_threshhold.shape[0]

		axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
		axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)

	plt.show()


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
	"""Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
	Note: Checks if pix is valid (in map)
	@param im_size - width, height of image
	@param pix - tuple with i, j in [0..W-1, 0..H-1]
	@param size_pix - size of pixel in meters
	@return x,y """
	if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
		raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

	return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
	"""Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
	Note: Checks if x_y is valid (in map)
	@param im_size - width, height of image
	@param x_y - tuple with x,y in meters
	@param size_pix - size of pixel in meters
	@return i, j (integers) """
	pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

	if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
		raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
	return pix


def is_reachable(im, pix):
	""" Is the pixel reachable, i.e., has a neighbor that is free?
	Used for
	@param im - the image
	@param pix - the pixel i,j"""

	# Returns True (the pixel is adjacent to a pixel that is free)
	#  False otherwise
	# You can use four or eight connected - eight will return more points

	return any([im[i] == 255 for i in path_planning.eight_connected(pix)])


def find_all_possible_goals(im):
	""" Find all of the places where you have a pixel that is unseen next to a pixel that is free
	It is probably easier to do this, THEN cull it down to some reasonable places to try
	This is because of noise in the map - there may be some isolated pixels
	@param im - thresholded image
	@return dictionary or list or binary image of possible pixels"""

	# All white pixels
	free_spaces = np.argwhere(im == 255)

	# All pixels connected to a white pixel (eight connected)
	all_connected = np.array([np.array(list(path_planning.eight_connected(i))) for i in free_spaces]).reshape(-1, 2)

	# Of all_connected, the pixels that are not white and not black (< 255 and > 0)
	unseen = np.array([(i, j) for i, j in all_connected if (im[i, j] < 255) & (im[i, j] > 0)])

	rospy.loginfo(f'Number of unseen: {len(unseen)}')
	if len(unseen) < 250:
		return None

	# White spaces adjacent to unseen spaces
	unseen_connected = np.array([list(path_planning.eight_connected(i)) for i in unseen]).reshape(-1, 2)

	possible_goals = np.array([(j, i) for i, j in unseen_connected if (im[i, j] == 255)])

	return possible_goals


def find_best_point(im, possible_points, robot_loc):
	""" Pick one of the unseen points to go to
	@param im - thresholded image
	@param possible_points - possible points to chose from
	@param robot_loc - location of the robot (in case you want to factor that in)
	"""

	points_connected = np.array([np.array(list(path_planning.eight_connected(i))) for i in possible_points])

	# count points that are unseen
	unseen = (im[points_connected[:, 0], points_connected[:, 1]] < 255) & (im[points_connected[:, 0], points_connected[:, 1]] > 0)
	unseen_counts = np.sum(unseen, axis=1)

	weights = unseen_counts / np.sum(unseen_counts)

	# Randomly choose a point based on weights
	selected_point_idx = np.random.choice(list(range(0, len(possible_points))), p=weights)

	return (possible_points[selected_point_idx][0], possible_points[selected_point_idx][1])


def find_waypoints(im, path):
	""" Place waypoints along the path
	@param im - the thresholded image
	@param path - the initial path
	@ return - a new path"""

	# Create a list of waypoints where the path changes direction
	waypoints = [path[-1]]
	for i in range(len(path) - 2, 0, -1):
		a1 = math.atan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0])
		a2 = math.atan2(path[i + 1][1] - path[i][1], path[i + 1][0] - path[i][0])

		if abs(a1 - a2) > math.pi/6 or i % 20 == 0:
			waypoints.append(path[i])

	waypoints.append(path[0])

	return waypoints