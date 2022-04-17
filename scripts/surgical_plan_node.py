#!/usr/bin/env python
# license removed for brevity
from re import L
import rospy
from geometry_msgs.msg import Pose
import numpy as np
import copy
import open3d as o3d
import tf


class SurgicalPlanNode():


    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
    def listener(self):

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/camera_to_pelvis", Pose, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    def pick_points(pcd):
        print("")
        print(
            "1) Please pick at least three correspondences using [shift + left click]"
        )
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        print("")
        return vis.get_picked_points()
            
    def reaming_end_point_pub(self, transformation):
        
        source = o3d.io.read_point_cloud("../data/hip_1_stitched.ply")
        # pick points from two point clouds and builds correspondences
        picked_id_source = self.pick_points(source)
        corr = np.zeros((len(picked_id_source), 1)) 
        corr[:, 0] = picked_id_source
        
        

if __name__ == '__main__':
    
    sp = SurgicalPlanNode()
    