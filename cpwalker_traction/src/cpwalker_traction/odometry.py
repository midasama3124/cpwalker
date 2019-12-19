#!/usr/bin/python
import math
from math import sin, cos, pi

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

class WalkerOdometry(object):
    def __init__(self):
        self.rospy = rospy

        # inicializar as variaveis de odometria como zero
        self.init_params()
        self.init_variables_()
        self.init_pubs_()

        self.last_time = rospy.Time.now()

    def init_params(self):
        self.frame_id = self.rospy.get_param("traction_odometry/frame_id", "odom_raw")
        self.child_frame_id = self.rospy.get_param("traction_odometry/child_frame_id", "base_link")

    def init_variables_(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0

        self.vel_lin = 0.0
        self.vel_ang = 0.0

    def init_pubs_(self):
        # Let's initialize the publisher and the odom msg
        self.odom_pub = self.rospy.Publisher('odom_raw', Odometry, queue_size=100)
        self.odom = Odometry()
        self.odom.header.frame_id = self.frame_id # "odom"
        self.odom.child_frame_id = self.child_frame_id  ## "base_link"
        # Let's initialize the transform broadcaster and the message to broadcast
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_transform = TransformStamped()
        self.odom_transform.header.frame_id = self.frame_id
        self.odom_transform.child_frame_id = self.child_frame_id   

    def update_velocity(self,vel_linear_x,vel_angular_z):
        self.vel_lin = vel_linear_x
        self.vel_ang = vel_angular_z

    def update_odometry(self):
        self.current_time = self.rospy.Time.now()

        dt = (self.current_time - self.last_time).to_sec()
        delta_x = self.vel_lin * cos(self.pos_theta) * dt
        delta_y = self.vel_lin * sin(self.pos_theta) * dt
        delta_th = self.vel_ang * dt

        self.pos_x += delta_x
        self.pos_y += delta_y
        self.pos_theta += delta_th

        self.last_time = self.current_time

    def publish_odometry(self):
        odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.pos_theta)

        self.odom_transform.header.stamp = self.rospy.Time.now()
        self.odom_transform.transform.translation.x = self.pos_x
        self.odom_transform.transform.translation.y = self.pos_y
        self.odom_transform.transform.translation.z = 0.0
        self.odom_transform.transform.rotation.x = odom_quat[0]
        self.odom_transform.transform.rotation.y = odom_quat[1]
        self.odom_transform.transform.rotation.z = odom_quat[2]
        self.odom_transform.transform.rotation.w = odom_quat[3]

        self.odom_broadcaster.sendTransform(self.odom_transform)
        
        # Publish over topic
        self.odom.header.stamp = self.current_time
        self.odom.pose.pose = Pose(Point(self.pos_x, self.pos_y, 0.), Quaternion(*odom_quat))
        self.odom.twist.twist = Twist(Vector3(self.vel_lin, 0, 0), Vector3(0, 0, self.vel_ang))
        self.odom_pub.publish(self.odom)
        