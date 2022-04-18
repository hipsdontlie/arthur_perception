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



class Registration:
    
    def __init__(self, voxel_size):  
        self.voxel_size = voxel_size  

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
            o3d.io.write_point_cloud("/home/mrsd-team-c/arthur_ws/src/arthur_perception/pointclouds/collected_pc.ply", self.o3dpc)
            # self.pick_points(self.o3dpc)
            self.acquired_pointcloud = True
            # print(np.asarray(self.o3dpc.points))
            self.chosen_point = self.manual_registration(self.o3dpc)
            
            # self.broadcast_tf_reaming_end_pt(self.chosen_point)

            self.end_point_publisher(self.chosen_point)

            print("Done publishing end point!")
            rospy.set_param("save_pointcloud", False)

    def listener(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.set_param("save_pointcloud", False)
        print("inside listener")
        rospy.Subscriber("pelvis_pointcloud", PointCloud2, self.callback)
        rospy.spin()

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_transformed = source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_transformed, target_temp])    

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
        
        source = o3d.io.read_point_cloud("/home/mrsd-team-c/arthur_ws/src/arthur_perception/pointclouds/hip_1_stitched.ply")
        picked_id_source = self.pick_points(source)
        chosen_point = np.asarray(source.points)[picked_id_source]
        chosen_point = np.insert(chosen_point,3,1)
        print("chosen point", chosen_point)
        print("Transformation: ", final_transformation )
        transformed_point = np.matmul(final_transformation, chosen_point)
        print("transformed_point", transformed_point)
        return self.point_to_pose(transformed_point)


    def preprocess_point_cloud(self, pcd, voxel_size):
        print(":: Downsample with a voxel size %.3f." % self.voxel_size)
        pcd_down = pcd.voxel_down_sample(self.voxel_size)

        radius_normal = self.voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = self.voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def refine_registration(self, source, target, source_fpfh, target_fpfh,reg_p2p):
        distance_threshold = self.voxel_size * 0.4
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)
        result = o3d.registration.registration_icp(
            source, target, distance_threshold, reg_p2p.transformation,
            o3d.registration.TransformationEstimationPointToPlane())
        return result

    #global registration 
    def execute_global_registration(self,source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
        distance_threshold = self.voxel_size * 1.5
        print(":: RANSAC registration on downsampled point clouds.")
        print("   Since the downsampling voxel size is %.3f," % self.voxel_size)
        print("   we use a liberal distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result
        
    def manual_registration(self, target=None):
        # print(np.asarray(target.points))
        

        source = o3d.io.read_point_cloud("/home/mrsd-team-c/arthur_ws/src/arthur_perception/pointclouds/hip_1_stitched.ply")
        threshold = 1
        if(rospy.get_param('testing') == True):
            target = o3d.io.read_point_cloud("/home/mrsd-team-c/arthur_ws/src/arthur_perception/pointclouds/collected_pc.ply")
            target_s = copy.deepcopy(target)

        else: 
            # t_s = target_s.scale(1000, center=target_s.get_center())
            target_s = copy.deepcopy(target) 
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
        threshold = 0.5 #0.1  # 3cm distance threshold
        reg_p2p = o3d.registration.registration_icp(
            source, target_s, threshold, trans_init,
            o3d.registration.TransformationEstimationPointToPoint())

    

        #downsample points and extract FPFH features 
        source_down, source_fpfh = self.preprocess_point_cloud(source, self.voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target_s, self.voxel_size)

        result_ransac = self.execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            self.voxel_size)
        #visualize result after global registration 
        self.draw_registration_result(source_down, target_down, result_ransac.transformation)

        #refine registration and visualize 
        result = self.refine_registration(source_down, target_down, source_fpfh, target_fpfh, reg_p2p) #result_ransac)
        self.draw_registration_result(source, target_s, result.transformation)
        
        #choose reaming end-point
        self.chosen_point = self.draw_registration_result_final(source, target_s, result.transformation)
        print("Transforamtion is : ",result.transformation)
        
        evaluation = o3d.registration.evaluate_registration(source, target_s, threshold, result.transformation)
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


    def testing_registration(self):

        print("inside testing registration")
        self.chosen_point = self.manual_registration()
        rospy.set_param("save_pointcloud", False)
        self.acquired_pointcloud = True
        self.end_point_publisher(self.chosen_point) 

    def get_transformation(self, source_frame, target_frame,
                       tf_cache_duration=2.0):
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

    def transform_point(self,transformation, point_wrt_source,target_frame='pelvis'):
        point_wrt_target = \
            tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),
                transformation)

        point_wrt_target.header.frame_id=target_frame
        return point_wrt_target

        
    def end_point_publisher(self,point):
        br = tf.TransformBroadcaster()
        first_point = True
        listener = tf.TransformListener()
        pub = rospy.Publisher('reaming_end_point', PoseStamped, queue_size=10)
        print("Inside end point pub")
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            #br.sendTransform((point[0][0]/1000, point[0][1]/1000, point[0][2]/1000),(0,0,0,1),rospy.Time.now(),"initial_reaming_end_point","camera")

            try:
                if (first_point):
                    initial_pelvis_to_camera_tf = self.get_transformation('camera', 'pelvis')
                    print("Got first pelvis to camera TF!")
                    camera_to_reaming_pt = Point(point[0][0]/1000, point[0][1]/1000, point[0][2]/1000)
                    pelvis_to_reaming_end_point = self.transform_point(initial_pelvis_to_camera_tf, camera_to_reaming_pt)
                    first_point = False
                # (pelvis_camera_trans, pelvis_camera_rot) = listener.lookupTransform('camera', 'pelvis', rospy.Time(0))
                print("Updated pelvis to camera TF!")
                br.sendTransform((pelvis_to_reaming_end_point.point.x, pelvis_to_reaming_end_point.point.y, pelvis_to_reaming_end_point.point.z),(0.440, 0.762, -0.339, -0.332),rospy.Time.now(),"reaming_end_point","pelvis")
                print("Sent Pelvis to Reaming end-point transformation")
                (trans,rot) = listener.lookupTransform('base_link', 'reaming_end_point', rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("tf exception")
                continue
            

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

rospy.init_node('registration_node_v2', anonymous=True)
print("Started registration node")
reg = Registration(voxel_size = 0.1)
if(rospy.get_param('testing')==False):
    reg.listener()
else:
    reg.testing_registration()
