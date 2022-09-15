import open3d as o3d
import argparse
from pathlib import Path
from datetime import datetime, timedelta

from yaml import parse

now = datetime.today()
print('\nRUNNING STL PCD CONVERTER', now)
parser = argparse.ArgumentParser(description='Script to convert STL file to PCD.')
parser.add_argument('-p', '--path', required = True, dest = "path", help = "path to .stl file.", metavar='')
parser.add_argument('-o', '--output_dir', dest = "output_dir", help = "path to output directory [default: /home/$USER]", metavar='', default=str(Path.home()))
parser.add_argument('-v', '--visualize-pointcloud', dest = "viz", action='store_true', help = "visualize pointcloud", default=False)
args = parser.parse_args()

path = args.path

print('Loaded STL file from ', path)
print('\nSaving PCD file to ', args.output_dir)
mesh = o3d.io.read_triangle_mesh(path)
pointcloud = mesh.sample_points_poisson_disk(100000)

if args.viz:
    o3d.visualization.draw_geometries([pointcloud])

filename = '/converted_stl_pcd_' + str(now).replace(':','_').replace(' ', '_')[:16] + '.pcd'
output_path = args.output_dir + filename
print('Filename ', filename)
print(output_path)
o3d.io.write_point_cloud(output_path, pointcloud)

