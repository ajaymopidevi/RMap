#!/usr/bin/env python3

import rosbag
import rospy
import numpy as np
import os
from array import array
import struct
import math
import matplotlib.pyplot as plt
import tf 
import pcl
from pyntcloud import PyntCloud
import pandas as pd 

index = 0
basedir = '/home/ajay/catkin_ws_rmap/coloradar'
bag_file = 'ec_hallways_run0.bag'

radar_bag = rosbag.Bag(os.path.join(basedir,bag_file))
# Iterate over the lang_full messages

bagdir = os.path.join(basedir, bag_file.split('.bag')[0])
if not os.path.exists(bagdir):
    os.makedirs(bagdir)

radar_dir = os.path.join(bagdir, 'radar')
if not os.path.exists(radar_dir):
    os.makedirs(radar_dir)

radar_heatmap_dir = os.path.join(radar_dir, 'heatmap')
if not os.path.exists(radar_heatmap_dir):
    os.makedirs(radar_heatmap_dir)

radar_pcd_dir = os.path.join(radar_dir, "PCD") 
if not os.path.exists(radar_pcd_dir):
    os.makedirs(radar_pcd_dir)

def polar2cartesian(r,a,e):
    x = r * math.cos(e) * math.cos(a)
    y = r * math.cos(e) * math.sin(a)
    z = r * math.sin(e)
    return (x,y,z)

total_points = []

gt_times=[]
gt_poses = []
for topic, msg, time in radar_bag.read_messages(topics=["/lidar_ground_truth"]):
    # Start a new output file
    ts = time.to_sec()
    gt_times.append(ts)
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    
    euler = tf.transformations.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    pose = [
            position.x, position.y, position.z,
            euler[0], euler[1], euler[2]
            ]
    gt_poses.append(pose)
gt_times = np.array(gt_times)


for topic, msg, time in radar_bag.read_messages(topics=["/cascade/heatmap"]):
    # Start a new output file
    depth = msg.depth
    height = msg.height
    width = msg.width
    num_doppler_bins = msg.num_doppler_bins
    range_bin_width = msg.range_bin_width
    doppler_bin_width = msg.doppler_bin_width
    azimuth_bins = np.array(msg.azimuth_bins, dtype=float)
    elevation_bins = np.array(msg.elevation_bins, dtype=float)
    image = np.array(msg.image, dtype=float)
    #print(
        #"\n Depth: ",depth, 
        #"\n Height: ", height, 
        #"\n Width: ", width, 
        #"\n num_doppler_bins:", num_doppler_bins, 
        #"\n range_bin_width:", range_bin_width, 
        #"\n doppler_bin_width:", doppler_bin_width, 
        #"\n azimuth_bins:", azimuth_bins, 
        #"\n elevation_bins:", elevation_bins
    #    )
    # frame_vals = frame_vals[:-1:2] + 1j * frame_vals[1::2]

    file_index = '{:03d}'.format(index)
    intensity_filename = os.path.join(radar_heatmap_dir, "intensity_EAR_"+file_index+".npy")
    velocity_filename = os.path.join(radar_heatmap_dir, "velocity_EAR_"+file_index+".npy")
    image = image.reshape((elevation_bins.shape[0], azimuth_bins.shape[0], width,2))
    intensity = image[:,:,:,0]
    velocity = image[:,:,:,1]
    arr_shape = intensity.shape
    min_intensity = np.min(intensity)
    max_intensity = np.max(intensity)
    threshold = 0.1*max_intensity
    points = []
    for i in range(arr_shape[0]):
        for j in range(arr_shape[1]):
            for k in range(arr_shape[2]):
                if (intensity[i,j,k]>threshold):
                    e = elevation_bins[i]
                    a = azimuth_bins[j]
                    r = k * range_bin_width
                    pt = polar2cartesian(e,a,r)
                    points.append([pt[0],pt[1],pt[2],intensity[i,j,k],velocity[i,j,k]])
    points = np.array(points)
    df_points = pd.DataFrame(points, columns=["x","y","z","I", "v"])
    cloud = PyntCloud(df_points)
    ply_filename = os.path.join(radar_pcd_dir, file_index+".ply")

    cloud.to_file(ply_filename, as_text=True)
    

    pcd_filename = os.path.join(radar_pcd_dir, "PCD_"+file_index+".npy")
    with open(pcd_filename, "wb") as f:
        np.save(f, points)

    index += 1
    
    total_points.append(points.shape[0])
    
    #Generate the pose
    time_idx = np.argmin(np.abs(gt_times - time.to_sec()))
    pose = gt_poses[time_idx]
    pose_filename = os.path.join(radar_pcd_dir, "pose_"+file_index+".npy")
    with open(pose_filename, "wb") as f:
        np.save(f, pose)
    print(file_index, points.shape, np.prod(intensity.shape), "Radar time: ", time.to_sec(), "GT time: ", gt_times[time_idx], "idx:", time_idx, "Diff time: ", gt_times[time_idx]-time.to_sec(), "\n Pose: ", pose)

        
total_points = np.array(total_points)
print("Min: ", np.min(total_points), np.max(total_points), np.mean(total_points), np.median(total_points))

# Close the bag files
radar_bag.close()


plt.plot(total_points)
plt.savefig(os.path.join(bagdir, "radar_points.png"))

plt.clf()
gt_poses = np.array(gt_poses)
print(gt_poses[:,0].shape)
plt.plot(gt_poses[:,0])
plt.savefig(os.path.join(bagdir, "GT_POSES_0.png"))

plt.clf()
plt.plot(gt_poses[:,1])
plt.savefig(os.path.join(bagdir, "GT_POSES_1.png"))

plt.clf()
plt.plot(gt_poses[:,2])
plt.savefig(os.path.join(bagdir, "GT_POSES_2.png"))

plt.clf()
plt.plot(gt_poses[:,3])
plt.savefig(os.path.join(bagdir, "GT_POSES_3.png"))

plt.clf()
plt.plot(gt_poses[:,4])
plt.savefig(os.path.join(bagdir, "GT_POSES_4.png"))

plt.clf()
plt.plot(gt_poses[:,5])
plt.savefig(os.path.join(bagdir, "GT_POSES_5.png"))
