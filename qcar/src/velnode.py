#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


lidar_distance = 0

def lidar_callback(scan):
	global lidar_distance

	# Find the minimum distance that is greater than zero
	non_zero_distances = [d for d in scan.ranges if d >= .32]
	if non_zero_distances:
		min_distance = min(non_zero_distances)
	else:
		min_distance = .32

	# Set the lidar distance to the minimum distance
	lidar_distance = min_distance



def velocity_callback(Vector3Stamped_msg):
	global lidar_distance
	user_command_msg = Vector3Stamped()
	user_command_msg.header = Vector3Stamped_msg.header
	print(f"Current lidar distance: {lidar_distance} meters")

	# Check if the lidar distance is less than 0.05m (5cm)
	if lidar_distance < 0.33:
		# Stop the robot for three seconds
		user_command_msg.vector.x = 0
		user_command_msg.vector.y = 0
		user_command_msg.vector.z = 0
		pub.publish(user_command_msg)
		rospy.sleep(3)

		# Reverse the robot's path for three seconds
		user_command_msg.vector.x = -Vector3Stamped_msg.vector.x
		user_command_msg.vector.y = -Vector3Stamped_msg.vector.y
		user_command_msg.vector.z = -Vector3Stamped_msg.vector.z
		pub.publish(user_command_msg)
		rospy.sleep(2)
	else:
		# Translate the velocity data to the user command
		user_command_msg.vector.x = Vector3Stamped_msg.vector.x
		user_command_msg.vector.y = Vector3Stamped_msg.vector.y
		user_command_msg.vector.z = Vector3Stamped_msg.vector.z

	print(user_command_msg.vector.x)

	# Publish the user command to the /qcar/user_command topic
	pub.publish(user_command_msg)

#def print_lidar_distance():
	#global lidar_distance
	# Print the current lidar distance
	#print(f"Current lidar distance: {lidar_distance} meters")

	# Calculate and print the average lidar distance
	#average_distance = sum(lidar_distances) / len(lidar_distances)
	#print(f"Average lidar distance: {average_distance} meters")

if __name__ == '__main__':
	rospy.init_node('velnode')
	# Subscribe to the lidar data on the /lidar topic	
	lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
	
	pub = rospy.Publisher('qcar/user_command', Vector3Stamped, queue_size=10)
	sub = rospy.Subscriber('velocity', Vector3Stamped, velocity_callback)


	while not rospy.is_shutdown():
		# Call the print_lidar_distance function when the node is shut down
		#rospy.on_shutdown(print_lidar_distance)
		rospy.sleep(0.5)

	rospy.spin()

