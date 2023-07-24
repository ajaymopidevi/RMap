import os 
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import rosbag 

import open3d as o3d

bagfile = "/home/ajay/catkin_ws_rmap/coloradar/ec_hallways_run0.bag"

pcd_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_RScan_16"
pcd_files = sorted(os.listdir(pcd_folder))
pcd_files = [f for f in pcd_files if f.endswith(".ply")]
out_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_RScan_16_PoseIntegrated_Inv"
if not os.path.exists(out_folder):
    os.makedirs(out_folder)
gt_odom = []
gt_ts = []
bag = rosbag.Bag(bagfile)

# for topic,msg,time in bag.read_messages(topics=["/lidar_ground_truth"]):
#     gt_ts.append(time)
#     pos = msg.pose.pose.position
#     quat = msg.pose.pose.orientation
    
    
#     T = np.array([pos.x, pos.y, pos.z])
#     quat =  [quat.x, quat.y, quat.z, quat.w]

#     rot = Rotation.from_quat(quat)
#     R = rot.as_matrix()

#     transformation_matrix = np.identity(4)
#     transformation_matrix[:3, :3] = R
#     transformation_matrix[:3, 3] = T

#     gt_odom.append(transformation_matrix)





# gt_ts = np.array(gt_ts)
index=0


"""
  -                                                                  
    header:                                                          
      seq: 0                                                         
      stamp:                                                         
        secs: 1608587614                                             
        nsecs: 666150902                                             
      frame_id: "imu_viz_link"                                       
    child_frame_id: "cascade_link"                                   
    transform:                                                       
      translation:                                                   
        x: 0.03                                                      
        y: 0.12                                                      
        z: -0.09                                                     
      rotation:                                                      
        x: 0.0                                                       
        y: 0.0                                                       
        z: 0.706825181105366                                         
        w: 0.7073882691671998                                        
---
---
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1608587617
        nsecs:  80447358
      frame_id: "imu_viz_link"
    child_frame_id: "base_radar_link"
    transform:
      translation:
        x: -0.145
        y: 0.09
        z: -0.025
      rotation:
        x: 0.0
        y: 0.0
        z: 0.706825181105366
        w: 0.7073882691671998
---
"""
T = np.array([-0.145, 0.09, -0.025])
quat =  [0.0, 0.0, 0.706825181105366, 0.7073882691671998]
#T = np.array([0.03, 0.12, -0.09])
#quat =  [0.0, 0.0, 0.706825181105366, 0.7073882691671998]

#T = np.array([0, 0, 0])
#quat =  [0.7071054514659225, 0.7071080488140459,  0.00020953506788372477,  0.00020953429821985765]

rot = Rotation.from_quat(quat)
R = rot.as_matrix()

transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = R
transformation_matrix[:3, 3] = T
transformation_matrix_inv = np.linalg.inv(transformation_matrix)
print(transformation_matrix_inv)
for topic, msg, time in bag.read_messages(topics=["/mmWaveDataHdl/RScan"]):
    #closest_idx = np.argmin(np.abs(gt_ts - time))

    #transformation_matrix = gt_odom[closest_idx]
    #print(pcd_files[index])
    print(msg.header.frame_id, index)

    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
    
    pcd_transform = pcd.transform(transformation_matrix_inv)
    if index==54:
        print(np.min(pcd_transform.points, axis=0), np.max(pcd_transform.points, axis=0))
    o3d.io.write_point_cloud(os.path.join(out_folder, pcd_files[index]), pcd_transform, True, True)

    index += 1


bag.close()


