import os 
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import rosbag 

import open3d as o3d

bagfile = "/home/ajay/catkin_ws_rmap/coloradar/outdoors_run0.bag"
out_bagfile = "/home/ajay/catkin_ws_rmap/coloradar/outdoor0.bag"
# pcd_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_RScan_16"
pcd_folder = "/home/ajay/catkin_ws_rmap/coloradar/outdoors_run0/lidar/original"
pcd_files = sorted(os.listdir(pcd_folder))
pcd_files = [f for f in pcd_files if f.endswith(".ply")]
# out_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_Lidar16x_GTPose"
# if not os.path.exists(out_folder):
#     os.makedirs(out_folder)
gt_odom = []
gt_ts = []


def pcd2msg(points, msg):
    cloud_msg = msg
    cloud_msg.height = 1
    cloud_msg.width = len(points)
    cloud_msg.fields = msg.fields[0:3]
    cloud_msg.point_step = 12
    
    cloud_msg.row_step = 12*len(points)
    cloud_msg.header.frame_id = "os1_sensor"
    # Convert the points to binary data with little-endian byte order
    cloud_msg.data = np.asarray(points, np.float32).byteswap().tostring()
    return cloud_msg



#bag = rosbag.Bag(bagfile)
#out_bag = rosbag.Bag(out_bagfile, 'w')
os1_sensor2lidar = {}
os1_sensor2lidar['translation'] = np.array([0,0,0.03618])
os1_sensor2lidar['quaternion'] = np.array([0,0,1,0])
os1_sensor2lidar['rotation'] = Rotation.from_quat(os1_sensor2lidar['quaternion']).as_matrix()
transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = os1_sensor2lidar['rotation']
transformation_matrix[:3, 3] = os1_sensor2lidar['translation']

os1_sensor2lidar['transformation'] = transformation_matrix
os1_sensor2lidar['inverse_transformation'] = np.linalg.inv(transformation_matrix)


index=0

with rosbag.Bag(bagfile, 'r') as input_bag:
    with rosbag.Bag(out_bagfile, 'w') as output_bag:
        for topic, msg, time in input_bag.read_messages():
            if topic == "/os1_cloud_node/points":
                pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
                pcd = pcd.transform(os1_sensor2lidar['inverse_transformation'])
                index += 1
                points = pcd.points
                pcd_msg = pcd2msg(points, msg)
                output_bag.write("/os1_cloud_node/points/original", pcd_msg, time)
                print(index)
            
            output_bag.write(topic, msg, time)

