import os
import sys
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np

pcd_dir = "/home/ajay/catkin_ws_rmap/coloradar/octomaps/pcd"
files = os.listdir(pcd_dir)
lidar_files = [os.path.join(pcd_dir, f) for f in files if f.endswith("lidar_filtered.ply")]
radar_files = [os.path.join(pcd_dir, f) for f in files if f.endswith("radar.ply")]

gt_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/gt"
input_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/input"

if not os.path.exists(gt_dir):
    os.makedirs(gt_dir)

if not os.path.exists(input_dir):
    os.makedirs(input_dir)


def calculate_resolution(points):
    # Compute the Euclidean distance between neighboring points
    kdtree = cKDTree(points)
    _, indices = kdtree.query(points, k=2)  # Find the nearest 2 points for each point
    distances = np.linalg.norm(points[indices[:, 1]] - points[indices[:, 0]], axis=1)

    # Calculate the average distance as the resolution
    resolution = np.mean(distances)
    #print("Resolution:",resolution, np.min(distances), np.max(distances))

    return resolution

def geodesic_growth(pcd, seed_point, num_points, save_filename):
    patch_pcd = o3d.geometry.PointCloud()
    
    # Create a KDTree from the mesh vertices
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    [k, idx, _] = kdtree.search_knn_vector_3d(seed_point, num_points)

    patch_points = np.asarray(pcd.points)[idx, :]
    """
    #Normalize
    centroid = np.mean(patch_points, axis=0)
    patch_points -= centroid
    max_distance = np.max(np.linalg.norm(patch_points, axis=1))
    if max_distance==0:
        print("Error")
        return 1,patch_pcd
    patch_points /= max_distance
    """
    patch_pcd.points = o3d.utility.Vector3dVector(patch_points)
    #resolution = calculate_resolution(patch_points)
    
    o3d.io.write_point_cloud(save_filename, patch_pcd, True, True)
    

    return 0, patch_pcd

def get_lidar_seed_points(points, seed_points):
    lidar_points = []
    for pt in seed_points:
        distances = np.linalg.norm(points - pt, axis=1)
        closest_idx = np.argmin(distances)
        lidar_points.append(points[closest_idx])
    lidar_points = np.array(lidar_points)
    return lidar_points

if __name__ == "__main__":
    num_seeds = 50
    num_points = 16384
    for f in radar_files:
        radar_pcd = o3d.io.read_point_cloud(f)
        lidar_filename = f.replace("radar.ply", "lidar_filtered.ply")
        lidar_pcd = o3d.io.read_point_cloud(lidar_filename)
        num_seeds = max(50, int(len(lidar_pcd.points)*25/16384))
        print(len(radar_pcd.points), num_seeds)
        print(len(lidar_pcd.points), lidar_filename)
        seed_indices = np.random.choice(np.asarray(radar_pcd.points).shape[0], num_seeds, replace=False)

        radar_seed_points = np.asarray(radar_pcd.points)[seed_indices]

        #Find closest point in Lidar for each of the seed point
        #lidar_seed_points = get_lidar_seed_points(lidar_pcd, radar_seed_points)

        for i in range(num_seeds):
            gt_filename =  f"{os.path.basename(lidar_filename).split('.')[0]}_{i:04d}.pcd"
            gt_filepath = os.path.join(gt_dir, gt_filename)
            input_filename =  f"{os.path.basename(f).split('.')[0]}_{i:04d}.pcd"
            input_filepath = os.path.join(input_dir, input_filename)
            #print(input_filepath, gt_filepath)
            #print(i, lidar_seed_points[i], radar_seed_points[i])
            _, _ = geodesic_growth(lidar_pcd, radar_seed_points[i], num_points, gt_filepath)
            _, _ = geodesic_growth(radar_pcd, radar_seed_points[i], int(num_points/4), input_filepath)
            
        

