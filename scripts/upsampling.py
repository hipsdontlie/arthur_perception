import open3d as o3d
import numpy as np
import sys

if __name__ == "__main__":

   
    pcd = o3d.io.read_point_cloud("../data/bunnyData.pts")
    o3d.visualization.draw_geometries([pcd])
    alpha = 0.5
    # print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    # o3d.io.write_triangle_mesh("upsampled_acetabulum.ply",mesh)
    o3d.visualization.draw_geometries([mesh])