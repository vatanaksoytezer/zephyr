#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import tf
from nav_msgs.msg import Odometry

def odom_handler(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 
        msg.pose.pose.position.z),(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, 
        msg.pose.pose.orientation.w,),rospy.Time.now(),'firefly/base_link', "odom")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/firefly/vi_sensor/ground_truth/odometry', Odometry, odom_handler)
    rospy.spin()