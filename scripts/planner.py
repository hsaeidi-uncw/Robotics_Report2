#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams 
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist

sphere_x = 0
sphere_y = 0
sphere_z = 0
sphere_radius = 0 

def get_sphere(data):	
	global sphere_x
	global sphere_y
	global sphere_z
	global sphere_radius
	
	sphere_x = data.xc
	sphere_y = data.yc
	sphere_z = data.zc
	sphere_radius = data.radius

def transform_coords(raw_x, raw_y, raw_z, radius):
	flag = False
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	q_rot = Quaternion()	
	while not flag:
	
		# try getting the most update transformation between the camera frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			flag = True
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
	
	# Define a point in the camera frame:
	pt_cam = tf2_geometry_msgs.PointStamped()
	pt_cam.header.frame_id = 'camera_color_optical_frame'
	pt_cam.header.stamp = rospy.get_rostime()
	
	pt_cam.point.x = sphere_x
	pt_cam.point.y = sphere_y
	pt_cam.point.z = sphere_z
	
	# Convert the 3D point to the base frame coordinates:
	pt_in_base = tfBuffer.transform(pt_cam,'base', rospy.Duration(1.0))
	
	# Get XYZ coordinates from new frame:
	x = pt_in_base.point.x
	y = pt_in_base.point.y
	z = pt_in_base.point.z
	
	# Return XYZ coords:
	return x, y, z, radius
	

# define a function that defines a new point
def add_point(linearX, linearY, linearZ, angularX, angularY, angularZ, plan):
		plan_point = Twist()
		
		plan_point.linear.x = linearX
		plan_point.linear.y = linearY
		plan_point.linear.z = linearZ
		plan_point.angular.x = angularX
		plan_point.angular.y = angularY
		plan_point.angular.z = angularZ
		
		plan.points.append(plan_point)
		

# Callback function for getting current robot position:

current_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
def get_current_pos(data):
	global current_pos
	
	current_pos[0] = data.linear.x
	current_pos[1] = data.linear.y
	current_pos[2] = data.linear.z
	current_pos[3] = data.angular.x
	current_pos[4] = data.angular.y
	current_pos[5] = data.angular.z

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# Add a subscriber for the sphere parameters:
	rospy.Subscriber('sphere_params', SphereParams, get_sphere)
	# Add a subscriber for the robot's curent position:
	rospy.Subscriber('/ur5e/toolpose', Twist, get_current_pos)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	# The values for the ball are only gotten once:
	valid = False
	
	while not valid:
	
		if 0 not in [sphere_x, sphere_y, sphere_z, sphere_radius]:
			valid = True
			
		# Transform 
		trans_coords = transform_coords(sphere_x, sphere_y, sphere_z, sphere_radius)
	
	print("Before tranformed: ")
	print("x: ", sphere_x, "y: ", sphere_y, "z: ", sphere_z, "radius: ", sphere_radius, "\n")
	
	# Define transformed coordinates:
	x = trans_coords[0]
	y = trans_coords[1]
	z = trans_coords[2]
	radius = trans_coords[3]
	
	print("Transformed:")
	print("x: ", x, "y: ", y, "z: ", z, "radius: ", radius, "\n")
	
	
	# define a plan variable
	plan = Plan()
	# add points to plan 
	add_point(-0.5, -0.133, 0.5, 3.14, 0.0, 1.57, plan)
	add_point(x, y, 0.5, 3.14, 0.0, 1.57, plan)
	add_point(x, y, (z)+radius, 3.14, 0.0, 1.57, plan)
	add_point(x, y, 0.5, 3.14, 0.0, 1.57, plan)
	add_point(-0.5, -0.133, 0.5, 3.14, 0.0, 1.57, plan)
	add_point(-0.5, -0.133, (z)+radius, 3.14, 0.0, 1.57, plan)

	while not rospy.is_shutdown():
		
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
