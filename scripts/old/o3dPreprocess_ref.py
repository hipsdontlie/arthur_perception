#!/usr/bin/env python3
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped,PointStamped
import numpy as np
import tf
import copy
import tf2_geometry_msgs


class Registration():
    
    
    def pick_points(self,pcd):
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

    
    def callback(self,data):
        self.acquired_pointcloud = False
        # print("inside callback")
        if rospy.get_param('save_pointcloud') == True: 
            self.o3dpc = orh.rospc_to_o3dpc(data)
            self.o3dpc_np = np.asarray(self.o3dpc.points) * 1000
            self.o3dpc = o3d.geometry.PointCloud()
            self.o3dpc.points = o3d.utility.Vector3dVector(self.o3dpc_np)
            o3d.visualization.draw_geometries([self.o3dpc])
            o3d.io.write_point_cloud("../pointclouds/collected_pc.ply", self.o3dpc)
            # self.pick_points(self.o3dpc)
            self.acquired_pointcloud = True
            # print(np.asarray(self.o3dpc.points))
            self.chosen_point = self.manual_registration(self.o3dpc)
            
            # self.broadcast_tf_reaming_end_pt(self.chosen_point)

            self.end_point_publisher(self.chosen_point)


            rospy.set_param("save_pointcloud", False)

    def listener(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.set_param("save_pointcloud", False)
        
        rospy.Subscriber("pelvis_pointcloud", PointCloud2, self.callback)
        rospy.spin()

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_transformed = source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])    

    def draw_registration_result_final(self,source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_transformed = source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])
        picked_id_source = self.pick_points(source_transformed)
        self.chosen_point = np.asarray(source_transformed.points)[picked_id_source]
        return self.chosen_point
        
    def get_end_point(self,final_transformation):
        
        source = o3d.io.read_point_cloud("../pointclouds/hip_1_stitched.ply")
        picked_id_source = self.pick_points(source)
        chosen_point = np.asarray(source.points)[picked_id_source]
        chosen_point = np.insert(chosen_point,3,1)
        print("chosen point", chosen_point)
        print("Transformation: ", final_transformation )
        transformed_point = np.matmul(final_transformation, chosen_point)
        print("transformed_point", transformed_point)
        return self.point_to_pose(transformed_point)
        
    def manual_registration(self, target):
        print(np.asarray(target.points))
        

        source = o3d.io.read_point_cloud("../pointclouds/hip_1_stitched.ply")
        threshold = 1
        
        # target = o3d.io.read_point_cloud("../pointclouds/collected_pc.ply")
        # target_s = copy.deepcopy(target)
        target_s = target
        # target_s = target_s.scale(1000, center=target_s.get_center()) 
        print("Visualization of two point clouds before manual alignment")
        # self.draw_registration_result(source, target_s, np.identity(4))

        # pick points from two point clouds and builds correspondences
        picked_id_source = self.pick_points(source)
        picked_id_target = self.pick_points(target_s)
        # print(np.asarray(target.points))
        assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
        assert (len(picked_id_source) == len(picked_id_target))
        corr = np.zeros((len(picked_id_source), 2))
        corr[:, 0] = picked_id_source
        corr[:, 1] = picked_id_target

        # estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(source, target_s,
                                                o3d.utility.Vector2iVector(corr))

        # point-to-point ICP for refinement
        print("Perform point-to-point ICP refinement")
        threshold = 0.1  # 3cm distance threshold
        reg_p2p = o3d.registration.registration_icp(
            source, target_s, threshold, trans_init,
            o3d.registration.TransformationEstimationPointToPoint())
        self.chosen_point = self.draw_registration_result_final(source, target_s, reg_p2p.transformation)
        print("Transforamtion is : ",reg_p2p.transformation)
        
        evaluation = o3d.registration.evaluate_registration(source, target_s, threshold, reg_p2p.transformation)
        final_transformation = reg_p2p.transformation
        print("Evaluation: ", evaluation)
        
        return self.chosen_point


    def point_to_pose(self,point):
        
        pose_msg = PoseStamped()
        print("Shape of point is", point.shape)
        print("Point coordinates are: ", point)
        pose_msg.header.frame_id = "camera"
        pose_msg.header.stamp = rospy.Time()
        pose_msg.pose.position.x = point[0][0]/1000
        pose_msg.pose.position.y = point[0][1]/1000
        pose_msg.pose.position.z = point[0][2]/1000

        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 1
        return pose_msg

                
            
    def convert_to_pose(self,final_transformation):
        
        print(final_transformation)
        pose_msg = PoseStamped() 
        pose_msg.position.x = final_transformation[0][3]
        pose_msg.position.y = final_transformation[1][3]
        pose_msg.position.z = final_transformation[2][3]
        
        q = tf.transformations.quaternion_from_matrix(final_transformation)
        
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
            
        return pose_msg

    def end_point_publisher(self,point):
        br = tf.TransformBroadcaster()
        listener = tf.TransformListener()
        pub = rospy.Publisher('reaming_end_point', PoseStamped, queue_size=10)
        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            
            # br.sendTransform((point[0][0]/1000, point[0][1]/1000, point[0][2]/1000),(0,0,0,1),rospy.Time.now(),"reaming_end_point","camera")

            try:

                pelvis_to_camera_tf = listener.lookupTransform('pelvis', 'camera', rospy.Time(0))
                camera_to_reaming_pt = PointStamped()
                camera_to_reaming_pt.header.stamp = rospy.Time()
                camera_to_reaming_pt.header.frame_id = 'camera'
                camera_to_reaming_pt.point.x = point[0][0]/1000
                camera_to_reaming_pt.point.y = point[0][1]/1000
                camera_to_reaming_pt.point.z = point[0][2]/1000

                pelvis_to_reaming_end_point = tf2_geometry_msgs.do_transform_point(camera_to_reaming_pt, pelvis_to_camera_tf)
                
                br.sendTransform((pelvis_to_reaming_end_point.point.x, pelvis_to_reaming_end_point.point.y, pelvis_to_reaming_end_point.point.z),(0,0,0,1),rospy.Time.now(),"reaming_end_point","camera")
                # (trans,rot) = listener.lookupTransform('base_link', 'initial_reaming_end_point', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            (trans,rot) = listener.lookupTransform('base_link', 'reaming_end_point', rospy.Time(0))
            pose_msg = PoseStamped()
            pose_msg.header.frame_id='base_link'
            pose_msg.header.stamp=rospy.Time()
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]

            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]

            pub.publish(pose_msg)
            rate.sleep()

rospy.init_node('registration_node', anonymous=True)
reg = Registration()
reg.listener()
