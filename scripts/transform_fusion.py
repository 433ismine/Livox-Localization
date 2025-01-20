#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import _thread
import copy
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None
cur_map_to_odom = None

def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom

    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)

        # TODO
        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        # br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
        #                  tf.transformations.quaternion_from_matrix(T_map_to_odom),
        #                  rospy.Time.now(),
        #                  'camera_init', 'map')
        if cur_odom is not None:
            # 发布全局定位的odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)
            # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map'
            localization.child_frame_id = 'body'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


def cb_delay_pub_init_pose(event):
    global cur_map_to_odom, init_map_to_odom
    cur_map_to_odom = init_map_to_odom


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')
    init_map_to_odom = Odometry()
    init_map_to_odom.pose.pose.position.x = rospy.get_param("/global_localization/pose_x")
    init_map_to_odom.pose.pose.position.y = rospy.get_param("/global_localization/pose_y")
    init_map_to_odom.pose.pose.position.z = rospy.get_param("/global_localization/pose_z")
    init_map_to_odom.pose.pose.orientation.x = rospy.get_param("/global_localization/rot_x")
    init_map_to_odom.pose.pose.orientation.y = rospy.get_param("/global_localization/rot_y")
    init_map_to_odom.pose.pose.orientation.z = rospy.get_param("/global_localization/rot_z")
    init_map_to_odom.pose.pose.orientation.w = rospy.get_param("/global_localization/rot_w")
    rospy.loginfo('Transform Fusion Node Inited... %s', init_map_to_odom)

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)
    rospy.Timer(rospy.Duration(6), cb_delay_pub_init_pose, True)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()
