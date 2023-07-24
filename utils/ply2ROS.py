import os 
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import rosbag 

import open3d as o3d

bagfile = "/home/ajay/catkin_ws_rmap/coloradar/ec_hallways_run0.bag"
out_bagfile = "/home/ajay/catkin_ws_rmap/coloradar/ec0_upsampled.bag"
pcd_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_RScan_16"
pcd_files = sorted(os.listdir(pcd_folder))
pcd_files = [f for f in pcd_files if f.endswith(".ply")]
out_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_RScan_16_PoseIntegrated_Inv"
if not os.path.exists(out_folder):
    os.makedirs(out_folder)
gt_odom = []
gt_ts = []


def pcd2msg(points, msg):
    cloud_msg = msg
    #print(cloud_msg.height, cloud_msg.width, cloud_msg.point_step, cloud_msg.row_step, cloud_msg.is_bigendian, cloud_msg.fields, cloud_msg.header)
    print(len(points), cloud_msg.is_dense)
    cloud_msg.width = len(points)
    cloud_msg.fields = msg.fields[0:3]
    cloud_msg.point_step = 12
    # Convert the points to binary data with little-endian byte order
    cloud_msg.data = np.asarray(points, np.float32).byteswap().tostring()
    return cloud_msg



#bag = rosbag.Bag(bagfile)
#out_bag = rosbag.Bag(out_bagfile, 'w')


index=0
"""
for topic, msg, time in bag.read_messages(topics=["/mmWaveDataHdl/RScan"]):
    #closest_idx = np.argmin(np.abs(gt_ts - time))

    #transformation_matrix = gt_odom[closest_idx]
    #print(pcd_files[index])
    print(msg.header.frame_id, index)

    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
    
    points = pcd.points

    pcd_msg = Pcd2msg(points, msg)
    break
    index += 1
"""
with rosbag.Bag(bagfile, 'r') as input_bag:
    with rosbag.Bag(out_bagfile, 'w') as output_bag:
        for topic, msg, time in input_bag.read_messages():
            if topic == "/mmWaveDataHdl/RScan":
                pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
                index += 1
                points = pcd.points
                pcd_msg = pcd2msg(points, msg)
                output_bag.write("/mmWaveDataHdl/RScan/upsampled", pcd_msg, time)
                print(index)
            output_bag.write(topic, msg, time)

