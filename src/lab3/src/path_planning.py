#!/usr/bin/env python3

# This assignment implements Dijkstra's shortest path on a graph, finding an unvisited node in a graph,
#   picking which one to visit, and taking a path in the map and generating waypoints along that path
#
# Given to you:
#   Priority queue
#   Image handling
#   Eight connected neighbors
#
# Slides https://docs.google.com/presentation/d/1XBPw2B2Bac-LcXH5kYN4hQLLLl_AMIgoowlrmPpTinA/edit?usp=sharing

# The ever-present numpy
import numpy as np
import rospy

# Our priority queue
import heapq

# Using imageio to read in the image
import imageio.v3 as imageio

import datetime

# -------------- Showing start and end and path ---------------
def plot_with_path(im, im_threshhold, zoom=1.0, robot_loc=None, goal_loc=None, path=None):
	"""Show the map plus, optionally, the robot location and goal location and proposed path
	@param im - the image of the SLAM map
	@param im_threshhold - the image of the SLAM map
	@param zoom - how much to zoom into the map (value between 0 and 1)
	@param robot_loc - the location of the robot in pixel coordinates
	@param goal_loc - the location of the goal in pixel coordinates
	@param path - the proposed path in pixel coordinates"""

	# Putting this in here to avoid messing up ROS
	import matplotlib.pyplot as plt

	fig, axs = plt.subplots(1, 2)
	axs[0].imshow(im, origin='lower', cmap="gist_gray")
	axs[0].set_title("original image")
	axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
	axs[1].set_title("threshold image")
	"""
	# Used to double check that the is_xxx routines work correctly
	for i in range(0, im_threshhold.shape[1]-1, 10):
		for j in range(0, im_threshhold.shape[0]-1, 10):
			if is_wall(im_thresh, (i, j)):
				axs[1].plot(i, j, '.b')
	"""

	# Double checking lower left corner
	axs[1].plot(10, 5, 'xy', markersize=5)

	# Show original and thresholded image
	for i in range(0, 2):
		if robot_loc is not None:
			axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
		if goal_loc is not None:
			axs[i].plot(goal_loc[0], goal_loc[1], '*g', markersize=10)
		if path is not None:
			for p, q in zip(path[0:-1], path[1:]):
				axs[i].plot([p[0], q[0]], [p[1], q[1]], '-y', markersize=2)
				axs[i].plot(p[0], p[1], '.y', markersize=2)
		axs[i].axis('equal')

	for i in range(0, 2):
		# Implements a zoom - set zoom to 1.0 if no zoom
		width = im.shape[1]
		height = im.shape[0]

		axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
		axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)

	plt.show()


# -------------- Thresholded image True/False ---------------
def is_wall(im, pix):
	""" Is the pixel a wall pixel?
	@param im - the image
	@param pix - the pixel i,j"""
	if im[pix[1], pix[0]] == 0:
		return True
	return False


def is_unseen(im, pix):
	""" Is the pixel one we've seen?
	@param im - the image
	@param pix - the pixel i,j"""
	if im[pix[1], pix[0]] == 128:
		return True
	return False


def is_free(im, pix):
	""" Is the pixel empty?
	@param im - the image
	@param pix - the pixel i,j"""
	# print(pix)
	if im[pix[1], pix[0]] == 255:
		return True
	return False


# def is_fattened_free(im, pix):
#     """ Are all pixels in an 8x8 square around pix free?
#     @param im - the image
#     @param pix - the pixel i,j"""

#     for i in range(-4, 4):
#         for j in range(-4, 4):
#             if is_wall(im, (pix[0] + i, pix[1] + j)):
#                 return False

#     return True


def convert_image(im, wall_threshold, free_threshold):
	""" Convert the image to a thresholded image with not seen pixels marked
	@param im - WXHX ?? image (depends on input)
	@param wall_threshold - number between 0 and 1 to indicate wall
	@param free_threshold - number between 0 and 1 to indicate free space
	@return an image of the same WXH but with 0 (free) 255 (wall) 128 (unseen)"""

	# Assume all is unseen
	im_ret = np.zeros((im.shape[0], im.shape[1]), dtype='uint8') + 128

	im_avg = im
	if len(im.shape) == 3:
		# RGB image - convert to gray scale
		im_avg = np.mean(im, axis=2)
	# Force into 0,1
	im_avg = im_avg / np.max(im_avg)
	# threshold
	#   in our example image, black is walls, white is free
	im_ret[im_avg > 1 - wall_threshold] = 0
	im_ret[im_avg < 1 - free_threshold] = 255
	im_ret[im == -1] = 128

	return im_ret


def fatten_image(im, pixels):
	""" Expand all walls by 4 pixels
	@param im - the image
	@return the fattened image"""

	w, h = im.shape

	im_ret = im.copy()

	all_wall = np.argwhere(im_ret == 0)

	for pix in all_wall:
		for i in range(-pixels, pixels):
			for j in range(-pixels, pixels):
				if pix[0] + j >= w or pix[1] + i >= h:
					continue

				im_ret[pix[0] + j, pix[1] + i] = 0

	return im_ret


# -------------- Getting 4 or 8 neighbors ---------------
def four_connected(pix):
	""" Generator function for 4 neighbors
	@param im - the image
	@param pix - the i, j location to iterate around"""
	for i in [-1, 1]:
		ret = pix[0] + i, pix[1]
		yield ret
	for i in [-1, 1]:
		ret = pix[0], pix[1] + i
		yield ret


def eight_connected(pix):
	""" Generator function for 8 neighbors
	@param im - the image
	@param pix - the i, j location to iterate around"""
	for i in range(-1, 2):
		for j in range(-1, 2):
			if i == 0 and j == 0:
				continue
			ret = pix[0] + i, pix[1] + j
			yield ret


def dijkstra(im, robot_loc, goal_loc):
	""" Occupancy grid image, with robot and goal loc as pixels
	@param im - the thresholded image - use is_free(i, j) to determine if in reachable node
	@param robot_loc - where the robot is (tuple, i,j)
	@param goal_loc - where to go to (tuple, i,j)
	@returns a list of tuples"""

	# Sanity check
	if not is_free(im, robot_loc):
		raise ValueError(f"Start location {robot_loc} is not in the free space of the map")

	if not is_free(im, goal_loc):
		raise ValueError(f"Goal location {goal_loc} is not in the free space of the map")

	# The priority queue itself is just a list, with elements of the form (weight, (i,j))
	#    - i.e., a tuple with the first element the weight/score, the second element a tuple with the pixel location
	priority_queue = []
	# Push the start node onto the queue
	#   push takes the queue itself, then a tuple with the first element the priority value and the second
	#   being whatever data you want to keep - in this case, the robot location, which is a tuple
	heapq.heappush(priority_queue, (0, robot_loc))

	# The power of dictionaries - we're going to use a dictionary to store every node we've visited, along
	#   with the node we came from and the current distance
	# This is easier than trying to get the distance from the heap
	visited = {}
	# Use the (i,j) tuple to index the dictionary
	#   Store the best distance, the parent, and if closed y/n
	visited[robot_loc] = (0, None, False)   # For every other node this will be the current_node, distance

	# While the list is not empty - use a break for if the node is the end node
	while priority_queue:
		# Get the current best node off of the list
		current_node = heapq.heappop(priority_queue)
		# Pop returns the value and the i, j
		node_score = current_node[0]
		node_ij = current_node[1]

		# Showing how to get this data back out of visited
		visited_triplet = visited[node_ij]
		visited_distance = visited_triplet[0]
		visited_parent = visited_triplet[1]
		visited_closed_yn = visited_triplet[2]

		# If we're at the goal node, break
		if node_ij == goal_loc:
			break

		# If this node is closed, skip it
		if visited_closed_yn:
			continue

		# Set the node to closed
		visited[node_ij] = (visited_distance, visited_parent, True)

		# Loop through neighbors
		for neighbor in eight_connected(node_ij):
			# Check if neighbor is a wall
			if not is_free(im, neighbor):
				continue

			# # Check if neighbor is a fattened wall
			# if not is_fattened_free(im, neighbor):
			#     continue

			edge_cost = 1 if neighbor[0] == node_ij[0] or neighbor[1] == node_ij[1] else np.sqrt(2)

			# Check if neighbor is in the visited dictionary
			if neighbor not in visited:
				# If not, add it to the dictionary
				visited[neighbor] = (node_score + edge_cost, node_ij, False)

				# Add the neighbor to the priority queue
				heapq.heappush(priority_queue, (node_score + edge_cost, neighbor))
			elif visited[neighbor][0] > node_score + edge_cost:
				visited[neighbor] = (node_score + edge_cost, node_ij, False)

	# Now check that we actually found the goal node
	try_2 = None
	if not goal_loc in visited:
		best = 1e30
		for v in visited.items():
			if v[0] is not None and v[0] < best:
				best = v[0]
				try_2 = v[1]
				rospy.loginfo(f'Trying again with {v}: {try_2}')

		if try_2 is not None:
			return dijkstra(im, robot_loc, try_2)

		raise ValueError(f"Goal {goal_loc} not reached")
		return []

	path = []
	path.append(goal_loc)

	current_node = goal_loc
	while current_node != robot_loc:
		current_node = visited[current_node][1]
		path.append(current_node)

	return path


def open_image(im_name):
	""" A helper function to open up the image and the yaml file and threshold
	@param im_name - name of image in Data directory
	@returns image anbd thresholded image"""

	# Needed for reading in map info
	from os import open

	im = imageio.imread("Data/" + im_name)

	wall_threshold = 0.7
	free_threshold = 0.9
	try:
		yaml_name = "Data/" + im_name[0:-3] + "yaml"
		with open(yaml_name, "r") as f:
			dict = yaml.load_all(f)
			wall_threshold = dict["occupied_thresh"]
			free_threshold = dict["free_thresh"]
	except:
		pass

	im_thresh = convert_image(im, wall_threshold, free_threshold)
	return im, im_thresh


if __name__ == '__main__':
	# Putting this here because in JN it's yaml
	import yaml

	# Use one of these

	""" Values for SLAM map
	im, im_thresh = open_image("SLAM_map.png")
	robot_start_loc = (200, 150)
	# Closer one to try
	# robot_goal_loc = (315, 250)
	robot_goal_loc = (615, 850)
	zoom = 0.8
	"""

	""" Values for map.pgm"""
	im, im_thresh = open_image("map.pgm")
	robot_start_loc = (1940, 1953)
	robot_goal_loc = (2135, 2045)
	zoom = 0.1

	# fattened_im_thresh = fatten_image(im_thresh)



	"""
	print(f"Image shape {im_thresh.shape}")
	for i in range(0, im_thresh.shape[1]-1):
		for j in range(0, im_thresh.shape[0]-1):
			if is_free(im_thresh, (i, j)):
				print(f"Free {i} {j}")
	"""

	before = datetime.datetime.now()
	path = dijkstra(fattened_im_thresh, robot_start_loc, robot_goal_loc)
	after = datetime.datetime.now()

	# print(f"Time to find path: {after - before}")

	plot_with_path(im, im_thresh, zoom=zoom, robot_loc=robot_start_loc, goal_loc=robot_goal_loc, path=path)

	print("Done")
