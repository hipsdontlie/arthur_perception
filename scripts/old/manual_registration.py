import numpy as np
import copy
import open3d as o3d


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


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


def demo_manual_registration():
    print("Demo for manual ICP")
    source = o3d.io.read_point_cloud("../data/hip_1_stitched.ply")
    threshold = 1
    
    target = o3d.io.read_point_cloud("../data/pelvis_collected_new.pcd")
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
    draw_registration_result(source, target_s, reg_p2p.transformation)
    print("Transforamtion is : ",reg_p2p.transformation)
    
    evaluation = o3d.registration.evaluate_registration(source, target_s, threshold, reg_p2p.transformation)
    print("Evaluation: ", evaluation)


if __name__ == "__main__":
    demo_manual_registration()