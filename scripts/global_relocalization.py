#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from livox_localization.srv import GlobalLocalization, GlobalLocalizationResponse

from scan_context_manager import ScanContextManager


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def process_cur_scan(pc_msg):
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'body'
    pc_msg.header.stamp = rospy.Time().now()

    # fastlio给的field有问题 处理一下
    # pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
    #                  pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
    #                  pc_msg.fields[3], pc_msg.fields[7]]
    print(len(pc_msg.fields))
    pc = msg_to_array(pc_msg)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pc[:, :3])
    return point_cloud


def do_global_localization(req):
    cur_scan = process_cur_scan(req.cloud_msg)
    initial_pose = livox_sc.initialization(np.asarray(cur_scan.points))
    if initial_pose is not None:
        rospy.loginfo('Global localization successfully!')
        pose_msg = PoseStamped()
        xyz = tf.transformations.translation_from_matrix(initial_pose)
        quat = tf.transformations.quaternion_from_matrix(initial_pose)
        pose_msg.pose = Pose(Point(*xyz), Quaternion(*quat))
        pose_msg.header.stamp = req.cloud_msg.header.stamp
        pose_msg.header.frame_id = 'map'
        resp = GlobalLocalizationResponse(pose_msg)
        return resp
    else:
        raise rospy.ServiceException("Failed to global localization")


if __name__ == '__main__':
    rospy.init_node("global_localization")

    # NOTE Modify your ScanContext file path HERE!
    file_path = "/home/dynamicx/rm_ws/src/rm_sentry/param/rm_navigation/global_localization"
    livox_sc = ScanContextManager(file_path=file_path)
    livox_sc.livox_load_sc_rk()
    server = rospy.Service("/global_localization", GlobalLocalization, do_global_localization)
    rospy.spin()
