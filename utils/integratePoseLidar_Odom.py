import os 
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import rosbag 

import open3d as o3d

bagfile = "/home/ajay/catkin_ws_rmap/OdomBeyondVision/2019-10-24-17-51-58.bag"

pcd_folder = "/home/ajay/catkin_ws_rmap/OdomBeyondVision/2019-10-24-17-51-58/lidar/original/"
pcd_files = sorted(os.listdir(pcd_folder))
pcd_files = [f for f in pcd_files if f.endswith(".ply")]
out_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_Lidar_PoseIntegrated_Inv_Odom"
if not os.path.exists(out_folder):
    os.makedirs(out_folder)


bag = rosbag.Bag(bagfile)

index=0

"""
---
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1571935925
        nsecs: 898688470
      frame_id: "/base_link"
    child_frame_id: "/velodyne"
    transform:
      translation:
        x: -0.05
        y: 0.0
        z: 0.523
      rotation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
"""

base2Lidar = {}
base2Lidar['translation'] = np.array([-0.05, 0.0, 0.523])
base2Lidar['quaternion'] =  np.array([0.0, 0.0, 0.0, 1])
base2Lidar['rotation'] = Rotation.from_quat(base2Lidar['quaternion']).as_matrix()

transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = base2Lidar['rotation']
transformation_matrix[:3, 3] = base2Lidar['translation']
base2Lidar['transformation'] = transformation_matrix
base2Lidar['inverse_transformation'] = np.linalg.inv(transformation_matrix)

for msg, topic, time in bag.read_messages(topics=["/velodyne_points"]):
    #closest_idx = np.argmin(np.abs(gt_ts - time))
    print(index)
    #transformation_matrix = gt_odom[closest_idx]

    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
    # transformation_matrix = np.matmul(os1_sensor2lidar['inverse_transformation'], imuLink2os1['inverse_transformation'])
    transformation_matrix = base2Lidar['inverse_transformation']
    pcd_transform = pcd.transform(transformation_matrix)

    o3d.io.write_point_cloud(os.path.join(out_folder, pcd_files[index]), pcd_transform, True, True)

    index += 1


bag.close()


