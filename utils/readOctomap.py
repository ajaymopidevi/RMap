import os 
import numpy as np
import rospy
import rosbag 
import sensor_msgs.point_cloud2 as pcd2
import open3d as o3d
import pandas as pd

from pyntcloud import PyntCloud

bagfolder = "/home/ajay/catkin_ws_rmap/coloradar/octomaps"
files = sorted(os.listdir(bagfolder))
bagfiles = [os.path.join(bagfolder, f) for f in files if f.endswith(".bag")]
out_folder = os.path.join(bagfolder, "pcd")
if not os.path.exists(out_folder):
   os.makedirs(out_folder)
pcd_topics = [
    "/lidar/octomap_point_cloud_centers",
    "/lidar_filtered/octomap_point_cloud_centers",
    "/mmWave/octomap_point_cloud_centers"
]

for bagfile in bagfiles:
    print(bagfile)

    bag = rosbag.Bag(bagfile)
    for topic, msg, time in bag.read_messages(topics=pcd_topics):
    
        points = [point[0:3] for point in pcd2.read_points(msg, skip_nans=True)]
        points = np.array(points)

        df_points = pd.DataFrame(points, columns=["x","y","z"])
        cloud = PyntCloud(df_points)

        out_filename = os.path.basename(bagfile).split('.bag')[0]
        if topic == "/lidar/octomap_point_cloud_centers":
            out_filename += "_lidar.ply"
        elif topic == "/lidar_filtered/octomap_point_cloud_centers":
            out_filename += "_lidar_filtered.ply"
        else:
            out_filename += "_radar.ply"
        print(out_filename)
        cloud.to_file(os.path.join(out_folder, out_filename), as_text=True)
        # o3d.io.write_point_cloud(os.path.join(out_folder, "os1_octomap.ply"), pcd, True, True)


    bag.close()


