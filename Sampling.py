#!/usr/bin/env python3

import rosbag
import rospy
import numpy as np
import os
from array import array
import struct
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcd2
import matplotlib.pyplot as pyplot
import matplotlib.pyplot as plt 


# Range, Azimuth and Elevation
max_range = 15
min_range = 0
max_azimuth = 60
min_azimuth = -60
max_elevation = 45
min_elevation = -45

#Calculate max and min in X,Y,Z
g_x_min = min_range
g_x_max = max_range
g_y_min = np.sin(np.deg2rad(min_azimuth))
g_y_max = np.sin(np.deg2rad(max_azimuth))
g_z_min = np.sin(np.deg2rad(min_elevation))
g_z_max = np.sin(np.deg2rad(max_elevation))



basedir = '/home/ajay/catkin_ws_rmap/coloradar'

bag_file = 'ec_hallways_run0.bag'

radar_bag = rosbag.Bag(os.path.join(basedir,bag_file))
# Iterate over the lang_full messages
tx_order = []
samples = []

bagdir = os.path.join(basedir, bag_file.split('.bag')[0])
if not os.path.exists(bagdir):
    os.makedirs(bagdir)

lidar_dir = os.path.join(bagdir, 'sampled_lidar')
if not os.path.exists(lidar_dir):
    os.makedirs(lidar_dir)

#plt.plot(gt_times) 
#plt.savefig('GT_timestamps.png')       

fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
]


out_bag = rosbag.Bag(os.path.join(basedir,'sampled_'+bag_file), 'w')
index = 0
for topic, msg, time in radar_bag.read_messages(topics=["/os1_cloud_node/points"]):
    ts = msg.header.stamp.to_sec()
    #print(ts, gt_times[index:index+10], gt_times.shape)
    #if ts in gt_times[index:index+10] or ts in gt_times[index-10:index]:    
    if True:    
        # Start a new output file
        
        pc = [point[0:4] for point in pcd2.read_points(msg, skip_nans=True)]
        
        sampled_pc = []
        for pt in pc:
            if pt[0]<=g_x_max and pt[0]>=g_x_min and pt[1]<=g_y_max and pt[1]>=g_y_min and pt[2]<=g_z_max and pt[2]>=g_z_min:
                sampled_pc.append(pt) 
        print(len(pc), len(sampled_pc), msg.height, msg.width, msg.point_step, msg.row_step)
        sampled_msg = msg
        #sampled_msg.width = len(sampled_pc)
        #sampled_msg.height = 1
        #sampled_msg.row_step = sampled_msg.point_step*len(sampled_pc)
        sampled_msg.fields = fields
        sampled_pc = np.array(sampled_pc).tolist()
        sampled_msg.data = []
        for point in pc:
            sampled_msg.data.extend(struct.pack('<ffff', *point))
        #sampled_msg.data = pcd2.create_cloud(msg.header, msg.fields, pc) 
        
        out_bag.write(topic, sampled_msg, time)
        """
        sampled_pc = np.array(sampled_pc)
        # frame_vals = frame_vals[:-1:2] + 1j * frame_vals[1::2]
        data_filename = os.path.join(lidar_dir, str(index)+".npy")
        with open(data_filename, "wb") as f:
            np.save(f, sampled_pc)
        """
    index += 1


# Close the bag files
radar_bag.close()

