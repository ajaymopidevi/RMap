import os
import sys
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np

pcd_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/edgar_output_norm"
scene_name = "arpg_lab"
voxel_size = 0.15
files = sorted(os.listdir(pcd_dir))

pcd_files = [os.path.join(pcd_dir, f) for f in files if f.endswith(".pcd") and scene_name in f]



if __name__ == "__main__":
    points = []
    for i,f in enumerate(pcd_files):
        print(i, f)
        pcd = o3d.io.read_point_cloud(f)
        points.extend(pcd.points)
    
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(points)
    print(len(scene_pcd.points))
    scene_pcd = scene_pcd.voxel_down_sample(voxel_size=voxel_size)
    print(len(scene_pcd.points))
    o3d.io.write_point_cloud(os.path.join(pcd_dir, scene_name+'.pcd'), scene_pcd, True, True)
            
        

