#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import random
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, FluidPressure
from geometry_msgs.msg import Point, Pose, Quaternion, TwistWithCovariance, Vector3, PoseWithCovariance, Twist

class zephyr_sensor_fusion(object):
	def __init__(self):
		rospy.init_node('zephyr_sensor_fusion')

		self.odom_pub = rospy.Publisher('/firefly/odom_fused', Odometry, queue_size = 1)
		self.odom_fused = Odometry()
		self.fuse()

	def pressure_callback(self,data):
		P = data.fluid_pressure
		P0 = 101325.0 # Air pressure at 0 meter In Pa
		T = 30.0 # In celcius degrees
		# Calculate Altitude and write it into z data
		h = ((((P0/P)**(1.0/5.257))-1)*(T+273.15))/(0.00605) - 0.09 # 0.09 bias error
		kP = 10.0/8.0 # altitude gain
		self.odom_fused.pose.pose.position.z = h*kP
		# altitude_string = "Altitude is %s" % h
		# rospy.loginfo(altitude_string)

	def imu_callback(self,data):
		self.odom_fused.pose.pose.orientation = data.orientation
		self.odom_fused.twist.twist.angular = data.angular_velocity

	def laser_callback(self,data):
		self.odom_fused.pose.pose.position.x = data.pose.pose.position.x
		self.odom_fused.pose.pose.position.y = data.pose.pose.position.y
		self.odom_fused.header = data.header
		self.odom_fused.child_frame_id = "firefly/base_link"
		self.odom_fused.twist.twist.linear = data.twist.twist.linear
		self.odom_fused.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	def fuse(self):
		rate = rospy.Rate(40) # 40hz
		while not rospy.is_shutdown():
			# Subscriber(s)
			rospy.Subscriber('/firefly/air_pressure', FluidPressure, self.pressure_callback)
			rospy.Subscriber('/firefly/imu', Imu, self.imu_callback)
			rospy.Subscriber('/scanmatch_odom', Odometry, self.laser_callback)
			# Publisher(s)
			self.odom_pub.publish(self.odom_fused)

			# Sleep until the next loop
			rate.sleep()

if __name__ == '__main__':
	zephyr_sensor_fusion()
