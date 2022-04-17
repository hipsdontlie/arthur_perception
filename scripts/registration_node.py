#!/usr/bin/env python
# license removed for brevity
from re import L
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import copy
import open3d as o3d
import tf

obtained_transformation = False

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_transformed = source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    
    

def draw_registration_result_final(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_transformed = source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    picked_id_source = pick_points(source_transformed)
    chosen_point = np.asarray(source_transformed.points)[picked_id_source]
    return point_to_pose(chosen_point)
    
def get_end_point(final_transformation):
    
    source = o3d.io.read_point_cloud("../pointclouds/hip_1_stitched.ply")
    picked_id_source = pick_points(source)
    chosen_point = np.asarray(source.points)[picked_id_source]
    chosen_point = np.insert(chosen_point,3,1)
    print("chosen point", chosen_point)
    print("Transformation: ", final_transformation )
    transformed_point = np.matmul(final_transformation, chosen_point)
    print("transformed_point", transformed_point)
    return point_to_pose(transformed_point)
    

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


def manual_registration():
    print("Demo for manual ICP")
    source = o3d.io.read_point_cloud("../pointclouds/hip_1_stitched.ply")
    threshold = 1
    
    target = o3d.io.read_point_cloud("../pointclouds/collected_pc.ply")
    target_s = copy.deepcopy(target)
    target_s = target_s.scale(1000, center=target_s.get_center()) 
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target_s, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target_s)
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
    chosen_point = draw_registration_result_final(source, target_s, reg_p2p.transformation)
    print("Transforamtion is : ",reg_p2p.transformation)
    
    evaluation = o3d.registration.evaluate_registration(source, target_s, threshold, reg_p2p.transformation)
    final_transformation = reg_p2p.transformation
    print("Evaluation: ", evaluation)
    
    return chosen_point


def point_to_pose(point):
    
    pose_msg = PoseStamped()
    
    pose_msg.header.frame_id = "camera"
    pose_msg.header.stamp = rospy.Time()
    
    pose_msg.pose.position.x = point[0]/1000
    pose_msg.pose.position.y = point[1]/1000
    pose_msg.pose.position.z = point[2]/1000

    pose_msg.pose.orientation.x = 0
    pose_msg.pose.orientation.y = 0
    pose_msg.pose.orientation.z = 0
    pose_msg.pose.orientation.w = 1
    
    return pose_msg
    

def convert_to_pose(final_transformation):
    
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

def end_point_publisher(pose_msg):
    pub = rospy.Publisher('reaming_end_point', PoseStamped, queue_size=10)
    rospy.init_node('registration_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    
        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        chosen_point = manual_registration()
        # end_point = get_end_point(final_transformation)
        end_point_publisher(chosen_point)
    except rospy.ROSInterruptException:
        pass
