#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to do some math
import numpy as np

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


# A callback to deal with the LaserScan messages.
def callback(scan):
	# Every time we get a laser scan, calculate the shortest scan distance in front
	# of the robot, and set the speed accordingly.  We assume that the robot is 38cm
	# wide.  This means that y-values with absolute values greater than 19cm are not
	# in front of the robot.  It also assumes that the LiDAR is at the front of the
	# robot (which it actually isn't) and that it's centered and pointing forwards.
	# We can work around these assumptions, but it's cleaner if we don't

	# Pulling out some useful values from scan
	angle_min = scan.angle_min
	angle_max = scan.angle_max
	num_readings = len(scan.ranges)

	# Doing this for you - get out theta values for each range/distance reading
	thetas = np.linspace(angle_min, angle_max, num_readings)

	# Get 1/2 width values using trig
	x = np.abs(scan.ranges * np.sin(thetas))

	# Find indecies less than 19cm
	x_infront = x <= 0.19

	# Get thetas and scan distances
	thetas_infront = thetas[x_infront]
	scans_infront = np.array(scan.ranges)[x_infront]

	# Find closest index
	idx_closest = np.argmin(scans_infront)

	# Create a twist and fill in all the fields (you will only set t.linear.x).
	t = Twist()
	t.linear.x = 0.0
	t.linear.y = 0.0
	t.linear.z = 0.0
	t.angular.x = 0.0
	t.angular.y = 0.0
	t.angular.z = 0.0

	# Get shortest scan distance
	shortest = scans_infront[idx_closest]

	# Adjust speed along sigmoid function
	x_speed = 1 / (1 + np.exp(5 - shortest)) * 8

	if shortest < 1.0:
		t.linear.x = 0   # Stop
	else:
		t.linear.x = x_speed   # Drive politely

	# Send the command to the robot.
	publisher.publish(t)

	# Print out a log message to the INFO channel to let us know it's working.
	rospy.loginfo(f'Shortest: {shortest} => {t.linear.x}')


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('stopper', argv=sys.argv)

	# Set up a publisher.  The default topic for Twist messages is cmd_vel.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
	subscriber = rospy.Subscriber('base_scan', LaserScan, callback, queue_size=10)

	# Now that everything is wired up, we just spin.
	rospy.spin()
