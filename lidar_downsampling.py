#!/usr/bin/env python3

import rosbag
import rospy
import numpy as np
import os
from array import array
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcd2
import matplotlib.pyplot as pyplot
import matplotlib.pyplot as plt 
from scipy.spatial import cKDTree
import pcl
from pyntcloud import PyntCloud
import pandas as pd 
import open3d as o3d


basedir = '/home/ajay/catkin_ws_rmap/coloradar'

bag_file = 'ec_hallways_run0.bag'

radar_bag = rosbag.Bag(os.path.join(basedir,bag_file))
# Iterate over the lang_full messages
tx_order = []
samples = []

bagdir = os.path.join(basedir, bag_file.split('.bag')[0])
if not os.path.exists(bagdir):
    os.makedirs(bagdir)

lidar_dir = os.path.join(bagdir, 'lidar')
if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)

lidar_original_dir = os.path.join(lidar_dir, "xyz_original")
if not os.path.exists(lidar_original_dir):
    os.makedirs(lidar_original_dir)

lidar_down2x_dir = os.path.join(lidar_dir, "xyz_down_2x")
if not os.path.exists(lidar_down2x_dir):
    os.makedirs(lidar_down2x_dir)

lidar_down4x_dir = os.path.join(lidar_dir, "xyz_down_4x")
if not os.path.exists(lidar_down4x_dir):
    os.makedirs(lidar_down4x_dir)

lidar_down8x_dir = os.path.join(lidar_dir, "xyz_down_8x")
if not os.path.exists(lidar_down8x_dir):
    os.makedirs(lidar_down8x_dir)

lidar_down16x_dir = os.path.join(lidar_dir, "xyz_down_16x")
if not os.path.exists(lidar_down16x_dir):
    os.makedirs(lidar_down16x_dir)


def calculate_resolution(points):
    # Compute the Euclidean distance between neighboring points
    kdtree = cKDTree(points)
    _, indices = kdtree.query(points, k=2)  # Find the nearest 2 points for each point
    distances = np.linalg.norm(points[indices[:, 1]] - points[indices[:, 0]], axis=1)

    # Calculate the average distance as the resolution
    resolution = np.mean(distances)

    return resolution

def downsample_pointcloud(cloud, leaf_size):
    # Create the VoxelGrid filter object
    #voxel_filter = cloud.make_voxel_grid_filter()

    # Set the leaf size (downsampling resolution)
    #voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)

    # Apply the downsampling filter
    #downsampled_cloud = voxel_filter.filter()
    downsampled_cloud = cloud.voxel_down_sample(leaf_size)
    return downsampled_cloud


index = 0
res = []
for topic, msg, time in radar_bag.read_messages(topics=["/os1_cloud_node/points"]):
    # Start a new output file
    pc = [point[0:4] for point in pcd2.read_points(msg, skip_nans=True)]
    pc = np.array(pc)
    ts = msg.header.stamp.to_sec()
    #resolution = calculate_resolution(pc[:,0:3])
    #print("Resolution: ", resolution)
    #res.append(resolution)
    print(pc.shape)
    file_index = '{:03d}'.format(index)
    df_points = pd.DataFrame(pc, columns=["x","y","z","I"])

    #Downsampling
    cloud = PyntCloud(df_points)
    pcd_filename = os.path.join(lidar_original_dir, file_index+".ply")

    cloud.to_file(pcd_filename, as_text=True)
    del cloud
    del df_points
    del pc

    pcd = o3d.io.read_point_cloud(pcd_filename)
    
    voxel_size = 0.04
    
    pcd_2x = pcd.voxel_down_sample(voxel_size*2)
    pcd_2x_filename = os.path.join(lidar_down2x_dir, file_index+".ply")
    o3d.io.write_point_cloud(pcd_2x_filename, pcd_2x, True, True)
    
    #voxel_size=0.01
    pcd_4x = pcd.voxel_down_sample(voxel_size*4)
    pcd_4x_filename = os.path.join(lidar_down4x_dir, file_index+".ply")
    o3d.io.write_point_cloud(pcd_4x_filename, pcd_4x, True, True)
    
    #voxel_size=0.5
    pcd_8x = pcd.voxel_down_sample(voxel_size*8)
    pcd_8x_filename = os.path.join(lidar_down8x_dir, file_index+".ply")
    o3d.io.write_point_cloud(pcd_8x_filename, pcd_8x, True, True)
    
    pcd_16x = pcd.voxel_down_sample(voxel_size*16)
    pcd_16x_filename = os.path.join(lidar_down16x_dir, file_index+".ply")
    o3d.io.write_point_cloud(pcd_16x_filename, pcd_16x, True, True)
    
    print(file_index, np.asarray(pcd_2x.points).shape, np.asarray(pcd_4x.points).shape, np.asarray(pcd_8x.points).shape, np.asarray(pcd_16x.points).shape)
    #data_filename = os.path.join(lidar_dir, str(index)+".npy")
    #with open(data_filename, "wb") as f:
    #   np.save(f, pc)
    # print(msg_samples.shape, msg_tx_order, index)
    index += 1

radar_bag.close()

plt.plot(res)
plt.savefig(os.path.join(lidar_dir, "resolutions.png"))

