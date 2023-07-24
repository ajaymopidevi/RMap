import os 
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import rosbag 

import open3d as o3d

bagfile = "/home/ajay/catkin_ws_rmap/OdomBeyondVision/2019-10-24-17-51-58.bag"

pcd_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/OdomBeyondVision_RScan_16"
pcd_files = sorted(os.listdir(pcd_folder))
pcd_files = [f for f in pcd_files if f.endswith(".ply")]
out_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/OdomBeyondVision_RScan_16_PoseIntegrated_Inv"
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
        nsecs: 969655181
      frame_id: "base_link"
    child_frame_id: "ti_mmwave_0"
    transform:
      translation:
        x: 0.16
        y: 0.0
        z: 0.3
      rotation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
---
"""
T = np.array([0.16, 0.0, 0.3])
quat =  [0.0, 0.0, 0.0, 1]

rot = Rotation.from_quat(quat)
R = rot.as_matrix()

transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = R
transformation_matrix[:3, 3] = T
transformation_matrix_inv = np.linalg.inv(transformation_matrix)
print(transformation_matrix_inv)
for topic, msg, time in bag.read_messages(topics=["/mmWaveDataHdl/RScan_middle"]):
    #closest_idx = np.argmin(np.abs(gt_ts - time))

    #transformation_matrix = gt_odom[closest_idx]
    #print(pcd_files[index])
    print(msg.header.frame_id, pcd_files[index])
    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
    
    pcd_transform = pcd.transform(transformation_matrix_inv)

    o3d.io.write_point_cloud(os.path.join(out_folder, pcd_files[index]), pcd_transform, True, True)

    index += 1


bag.close()


