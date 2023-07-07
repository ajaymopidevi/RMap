#!/usr/bin/env python3

import rosbag
import rospy
import numpy as np
import matplotlib.pyplot as plt

bag_file = "/home/ajay/catkin_ws_rmap/coloradar/ec_hallways_run0.bag"
topic_name = "/lidar_ground_truth"

# Open the bag file
bag = rosbag.Bag(bag_file)
x_pts = []
y_pts = []
# Iterate over the messages in the specified topic
for topic, msg, t in bag.read_messages(topics=[topic_name]):
    # Print the sentence
    pos = msg.pose.pose.position
    x_pts.append(pos.x)
    y_pts.append(pos.y)

x_pts = np.array(x_pts)
y_pts = np.array(y_pts)

plt.plot(x_pts, y_pts)
plt.savefig("odom_xy.png")
# Close the bag file
bag.close()

