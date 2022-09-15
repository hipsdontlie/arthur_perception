#!/usr/bin/env python3
from tracemalloc import get_traceback_limit
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped,PointStamped,Point
import numpy as np
import tf
import copy
import tf2_geometry_msgs
import tf2_ros

class OnlineCalibration():
    def get_transformation(self, source_frame, target_frame,tf_cache_duration=2.0):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
        tf2_ros.TransformListener(tf_buffer)

        # get the tf at first available time
        try:
            transformation = tf_buffer.lookup_transform(target_frame,
                    source_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from %s to %s'
                        % source_frame, target_frame)
        return transformation

    def ee_marker_pose_callback(self):

        self.camera_to_ee_marker_tf = self


    def listener(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.set_param("save_pointcloud", False)
        print("inside listener")
        rospy.Subscriber("", PointCloud2, self.callback)
        rospy.spin()


