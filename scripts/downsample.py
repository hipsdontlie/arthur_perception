import open3d as o3d
import numpy as np
import sys

if __name__ == "__main__":

   
    pcd = o3d.io.read_point_cloud("../data/acetabulum_real.ply")
    o3d.visualization.draw_geometries([pcd])
    downpcd = pcd.voxel_down_sample(voxel_size=1)
    o3d.io.write_point_cloud("../data/acetabulum_real_voxel_1.ply", downpcd)
    o3d.visualization.draw_geometries([downpcd])