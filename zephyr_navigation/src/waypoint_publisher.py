#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import random
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion, TwistWithCovariance, Vector3, PoseWithCovariance, Twist, Transform
from trajectory_msgs.msg import  MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class waypoint_publisher(object):
	def __init__(self):
		# Node(s)
		rospy.init_node('waypoint_publisher')

		# Var(s)
		self.kNanoSecondsInSecond = 1000000000
		self.new_room = 1
		self.wait_flag = 1
		self.front = 0
		self.right = 0
		self.left = 0
		self.back = 0
		self.current_odom = Odometry()
		self.current_scan = LaserScan()
		self.trajectory = MultiDOFJointTrajectory()

		# Publisher(s)
		# self.waypoint_pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)
		self.waypoint_pub = rospy.Publisher('/firefly/command/calculated_trajectory', MultiDOFJointTrajectory, queue_size = 10)

		# Function(s)
		self.waypoint_calculator()

	def laser_callback(self,data):
		self.current_scan = data

	def odom_callback(self,data):
		self.current_odom = data

	def waypoint_calculator(self):
		rate = rospy.Rate(40) # 40hz
		while not rospy.is_shutdown():
			# Subscriber(s)
			rospy.Subscriber('/firefly/scan', LaserScan, self.laser_callback)
			rospy.Subscriber('/firefly/odom_fused', Odometry, self.odom_callback)
			# TODO: Add new room subscriber

			if self.wait_flag:
				rospy.sleep(2.)
				self.wait_flag = 0			


			# Publisher(s)
			if self.new_room:
				# Loop laser scan ranges
				for i in range(0,45):
					self.front += self.current_scan.ranges[i]/90.0
				for i in range(315,360):
					self.front += self.current_scan.ranges[i]/90.0
				for i in range(45,135):
					self.right +=  self.current_scan.ranges[i]/90.0
				for i in range(135,225):
					self.back -=  self.current_scan.ranges[i]/90.0
				for i in range(225,315):
					self.left -=  self.current_scan.ranges[i]/90.0
				"""
				rospy.loginfo("Trajectory Type: %s",type(self.trajectory.points))
				rospy.loginfo("Trajectory Length: %s",len(self.trajectory.points))
				self.trajectory.points.time_from_start.nsecs = 2.0*self.kNanoSecondsInSecond
				self.trajectory.points.transforms.translation.x = (self.front+self.back)/2.0 # add odom
				self.trajectory.points.transforms.translation.y = (self.right+self.left)/2.0 # add odom
				self.trajectory.points.transforms.translation.z = 1.0
				self.trajectory.points.transforms.rotation.w = 1.0
				self.trajectory.header.stamp = rospy.Time.now()
				self.trajectory.joint_names = "firefly/base_link"
				"""
				self.waypoint_pub.publish(self.trajectory)
				"""
				rospy.loginfo("Published:\n Time:%f\tX:%f\tY:%f\tZ:%f\tYaw:%f\t", self.trajectory.points.time_from_start, 
					self.trajectory.points.transforms.translation.x, self.trajectory.points.transforms.translation.y, 
					self.trajectory.points.transforms.translation.z, self.trajectory.points.transforms.rotation.w)
				"""
				# self.new_room = 0

			# Sleep until the next loop
			rate.sleep()

if __name__ == '__main__':
	waypoint_publisher()
