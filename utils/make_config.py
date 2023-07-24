import os
import sys
import time
from datetime import datetime
import open3d as o3d
import math
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
import numpy as np

pcd_dir = "/home/ajay/catkin_ws_rmap/coloradar/octomaps/pcd"
files = os.listdir(pcd_dir)
lidar_files = [os.path.join(pcd_dir, f) for f in files if f.endswith("lidar_filtered.ply")]
radar_files = [os.path.join(pcd_dir, f) for f in files if f.endswith("radar.ply")]

train_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/train"
test_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/test"
val_dir = "/home/ajay/ARPG/RMap/data/ColoRadar/val"

if __name__ == "__main__":
    files = os.listdir(train_dir)