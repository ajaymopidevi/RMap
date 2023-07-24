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
import pandas as pd

import open3d as o3d

from pyntcloud import PyntCloud

basedir = '/home/ajay/catkin_ws_rmap/OdomBeyondVision'

bag_file = '2019-10-24-17-51-58.bag'

radar_bag = rosbag.Bag(os.path.join(basedir,bag_file))
# Iterate over the lang_full messages
tx_order = []
samples = []

bagdir = os.path.join(basedir, bag_file.split('.bag')[0])
if not os.path.exists(bagdir):
    os.makedirs(bagdir)


radar_dir = os.path.join(bagdir, 'radar')
if not os.path.exists(radar_dir):
    os.makedirs(radar_dir)

radar_dir = os.path.join(radar_dir, "RScan_PCD")
if not os.path.exists(radar_dir):
    os.makedirs(radar_dir)



index = 0
times = []
g_x_min = float('inf')
g_x_max = float('-inf')
g_y_min = float('inf')
g_y_max = float('-inf')
g_z_min = float('inf')
g_z_max = float('-inf')
for topic, msg, time in radar_bag.read_messages(topics=["/mmWaveDataHdl/RScan_middle"]):
    # Start a new output file
    pc = [point[0:3] for point in pcd2.read_points(msg, skip_nans=True)]
    pc = np.array(pc)
    x_min = np.min(pc[:,0])
    x_max = np.max(pc[:,0])
    y_min = np.min(pc[:,1])
    y_max = np.max(pc[:,1])
    z_min = np.min(pc[:,2])
    z_max = np.max(pc[:,2])
    if x_min < g_x_min:
        g_x_min = x_min
    if x_max > g_x_max:
        g_x_max = x_max
    if y_min < g_y_min:
        g_y_min = y_min
    if y_max > g_y_max:
        g_y_max = y_max
    if z_min < g_z_min:
        g_z_min = z_min
    if z_max > g_z_max:
        g_z_max = z_max
    #print("Min: ", x_min, y_min, z_min, "Max: ", x_max, y_max, z_max)
    #break
    print(pc.shape,index, msg.header.frame_id)
    ts = msg.header.stamp.to_sec()
    times.append(ts)
    # frame_vals = frame_vals[:-1:2] + 1j * frame_vals[1::2]
    #data_filename = os.path.join(radar_dir, str(index)+".npy")
    #with open(data_filename, "wb") as f:
    #   np.save(f, pc)
    # print(msg_samples.shape, msg_tx_order, index)
    
    df_points = pd.DataFrame(pc, columns=["x","y","z"])
    cloud = PyntCloud(df_points)
    file_index = '{:03d}'.format(index)
    pcd_filename = os.path.join(radar_dir, file_index+".ply")
    cloud.to_file(pcd_filename, as_text=True)
    index += 1
times = np.array(times)
#plt.plot(times) 
#plt.savefig('lidar_timestamps.png')       


# Close the bag files
radar_bag.close()

