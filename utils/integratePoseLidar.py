import os 
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import rosbag 

import open3d as o3d

bagfile = "/home/ajay/catkin_ws_rmap/coloradar/ec_hallways_run0.bag"

pcd_folder = "/home/ajay/catkin_ws_rmap/coloradar/ec_hallways_run0/lidar/original/"
pcd_files = sorted(os.listdir(pcd_folder))
pcd_files = [f for f in pcd_files if f.endswith(".ply")]
out_folder = "/home/ajay/ARPG/RMap/Grad-PU/output/2023-06-23T16:54:19.443319/test/ec_run0_radar_Lidar_PoseIntegrated_Inv"
if not os.path.exists(out_folder):
    os.makedirs(out_folder)
gt_odom = []
gt_ts = []
bag = rosbag.Bag(bagfile)
"""
for topic,msg,time in bag.read_messages(topics=["/lidar_ground_truth"]):
    gt_ts.append(time)
    pos = msg.pose.pose.position
    quat = msg.pose.pose.orientation
    
    
    T = np.array([pos.x, pos.y, pos.z])
    quat =  [quat.x, quat.y, quat.z, quat.w]

    rot = Rotation.from_quat(quat)
    R = rot.as_matrix()

    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = T

    gt_odom.append(transformation_matrix)

gt_ts = np.array(gt_ts)

"""
"""
---
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1608587617
        nsecs:  70531952
      frame_id: "imu_viz_link"
    child_frame_id: "os1_sensor"
    transform:
      translation:
        x: -0.075
        y: -0.02
        z: 0.0
      rotation:
        x: 0.0
        y: 0.0
        z: 0.6925369985634015
        w: 0.7213823574366062
---
 -                                                                         
    header:                                                                
      seq: 0                                                               
      stamp:                                                               
        secs: 1608587601                                                   
        nsecs: 734296923                                                   
      frame_id: "os1_sensor"                                                
    child_frame_id: "os1_lidar"                                             
    transform:                                                              
      translation:                                                          
        x: 0.0                                                              
        y: 0.0                                                              
        z: 0.03618                                                          
      rotation:                                                             
        x: 0.0                                                              
        y: 0.0                                                              
        z: 1.0                                                              
        w: 0.0
"""
os1_sensor2lidar = {}
os1_sensor2lidar['translation'] = np.array([0,0,0.03618])
os1_sensor2lidar['quaternion'] = np.array([0,0,1,0])
os1_sensor2lidar['rotation'] = Rotation.from_quat(os1_sensor2lidar['quaternion']).as_matrix()
transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = os1_sensor2lidar['rotation']
transformation_matrix[:3, 3] = os1_sensor2lidar['translation']

os1_sensor2lidar['transformation'] = transformation_matrix
os1_sensor2lidar['inverse_transformation'] = np.linalg.inv(transformation_matrix)

imuLink2os1 = {}
imuLink2os1['translation'] = np.array([-0.075, -0.02, 0.0])
imuLink2os1['quaternion'] =  np.array([0.0, 0.0, 0.6925369985634015, 0.7213823574366062])
imuLink2os1['rotation'] = Rotation.from_quat(imuLink2os1['quaternion']).as_matrix()

transformation_matrix = np.identity(4)
transformation_matrix[:3, :3] = imuLink2os1['rotation']
transformation_matrix[:3, 3] = imuLink2os1['translation']
imuLink2os1['transformation'] = transformation_matrix
imuLink2os1['inverse_transformation'] = np.linalg.inv(transformation_matrix)
index=0
for msg, topic, time in bag.read_messages(topics=["/os1_cloud_node/points"]):
    #closest_idx = np.argmin(np.abs(gt_ts - time))

    #transformation_matrix = gt_odom[closest_idx]

    pcd = o3d.io.read_point_cloud(os.path.join(pcd_folder, pcd_files[index]))
    # transformation_matrix = np.matmul(os1_sensor2lidar['inverse_transformation'], imuLink2os1['inverse_transformation'])
    # transformation_matrix = imuLink2os1['inverse_transformation']
    pcd_transform = pcd.transform(os1_sensor2lidar['inverse_transformation'])
    pcd_transform = pcd_transform.transform(imuLink2os1['inverse_transformation'])
    if index==54:
        print(np.min(pcd_transform.points, axis=0), np.max(pcd_transform.points, axis=0))
    
    o3d.io.write_point_cloud(os.path.join(out_folder, pcd_files[index]), pcd_transform, True, True)

    index += 1


bag.close()


