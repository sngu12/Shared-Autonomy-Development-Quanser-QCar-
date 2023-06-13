#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
#from control.pid import PID

#kp = 0.1
#ki =0.01
#kd = 0.01
#pid = PID(kp, ki, kd)

#setpoint = 1



def velocity_callback(Vector3Stamped_msg):

	#global lidar_distance
	user_command_msg = Vector3Stamped()
	user_command_msg.header = Vector3Stamped_msg.header

 # Compute the error between the setpoint and the actual linear velocity in the x direction
	#error = setpoint-Vector3Stamped_msg.vector.x
        
        # Compute the PID control output
	#control_output = pid(error)

	# Translate the velocity data to the user command
	#user_command_msg.vector.x = control_output
	user_command_msg.vector.x = Vector3Stamped_msg.vector.x
	user_command_msg.vector.y = Vector3Stamped_msg.vector.y
	user_command_msg.vector.z = Vector3Stamped_msg.vector.z
	
	# Publish the user command to the /qcar/user_command topic
	pub.publish(user_command_msg)



if __name__ == '__main__':
	rospy.init_node('velnode')
	#node2 = rospy.Node('lidarnode')

	pub = rospy.Publisher('qcar/user_command', Vector3Stamped, queue_size=10)
	sub = rospy.Subscriber('velocity', Vector3Stamped, velocity_callback)

	while not rospy.is_shutdown():
		rospy.sleep(0.5)

	rospy.spin()

