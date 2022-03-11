import numpy as np
import open3d as o3d
if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    # pcd = o3d.io.read_point_cloud("../../arthur_description/meshes/pelvis/pelvis_cropped.ply")
    
    pcd = o3d.io.read_point_cloud("../data/hip_2_stitched.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud!")
    downpcd = pcd.voxel_down_sample(voxel_size=1)
    print("Number of points is: ", len(downpcd.points))
    o3d.visualization.draw_geometries([downpcd])

    o3d.io.write_point_cloud("../data/pelvis_cropped_5.pcd", downpcd)