#!/usr/bin/env python3

import rospy
import struct
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry


# Range, Azimuth and Elevation
max_range = 15
min_range = 0
max_azimuth = 60
min_azimuth = -60
max_elevation = 45
min_elevation = -45

#Calculate max and min in X,Y,Z


class Sampling:
    def __init__(self):
        rospy.Subscriber("/os1_cloud_node/points", PointCloud2, self.sampling_callback)
        #rospy.Subscriber("/lidar_ground_truth", Odometry, self.update_callback)
        # Initialize ROS node
        #rospy.init_node('pointcloud_publisher', anonymous=True)
        
        # Create a publisher for the PointCloud2 topic
        self.pub = rospy.Publisher('/os1_cloud_node/sampled_points', PointCloud2, queue_size=10)
        self.g_x_min = min_range
        self.g_x_max = max_range
        self.g_y_min = np.sin(np.deg2rad(min_azimuth))*max_range
        self.g_y_max = np.sin(np.deg2rad(max_azimuth))*max_range
        self.g_z_min = np.sin(np.deg2rad(min_elevation))*max_range
        self.g_z_max = np.sin(np.deg2rad(max_elevation))*max_range
    
    def sampling_callback(self, msg):
        sampled_msg = msg
        #pcd_gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "t", "reflectivity", "ring", "noise", "range"), skip_nans=True)
        pcd_gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        pcd = list(pcd_gen)
        sampled_pc = []
        for pt in pcd:
            if pt[0]<=self.g_x_max and pt[0]>=self.g_x_min and pt[1]<=self.g_y_max and pt[1]>=self.g_y_min and pt[2]<=self.g_z_max and pt[2]>=self.g_z_min:
                sampled_pc.append(pt)
        sampled_pc = np.array(sampled_pc, dtype=np.float32) 
        #print(sampled_msg.header.frame_id, sampled_pc.shape)
        self.publish_pcd(sampled_pc, sampled_msg)
    
    def update_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        euler = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
        azimuth = euler[2] * 180.0 / 3.14159  # Convert radians to degrees

        # Calculate the elevation angle (pitch) in degrees
        elevation = euler[1] * 180.0 / 3.14159  # Convert radians to degrees

        roll = euler[0] * 180.0 / 3.14159  # Convert radians to degrees

        # Print the azimuth and elevation angles
        print("Azimuth: {:.2f} degrees".format(azimuth))
        print("Elevation: {:.2f} degrees".format(elevation))
        print("Roll: {:.2f} degrees".format(roll))
        
        #self.g_y_min = np.sin(np.deg2rad(azimuth+min_azimuth))*max_range
        #self.g_y_max = np.sin(np.deg2rad(azimuth+max_azimuth))*max_range
        #self.g_z_min = np.sin(np.deg2rad(elevation+min_elevation))*max_range
        #self.g_z_max = np.sin(np.deg2rad(elevation+max_elevation))*max_range
        

        
    
    def publish_pcd(self, sampled_pc, sampled_msg):
        sampled_msg.fields = sampled_msg.fields[0:4]
        
        sampled_msg.fields[-1].offset = 12
        #print(sampled_msg.fields[-1].offset)
        sampled_msg.point_step = 16
        sampled_msg.row_step = sampled_msg.point_step * 4
        sampled_msg.width = sampled_pc.shape[0]
        sampled_msg.height = 1
        sampled_pc = sampled_pc.reshape(-1)
        sampled_msg.data = sampled_pc.tobytes()
        self.pub.publish(sampled_msg)

    
    

        


    def start(self):
        rospy.init_node('pointcloud_subscriber', anonymous=True)
        rate = rospy.Rate(10)
        #while not rospy.is_shutdown:
        #    rate.sleep()
        rospy.spin()
    
        
    
    


if __name__ == '__main__':
    """
    try:
        rospy.Subscriber("/os1_cloud_node/points", PointCloud2, sampling_callback)
        # Initialize ROS node
        rospy.init_node('pointcloud_publisher', anonymous=True)
        
        # Create a publisher for the PointCloud2 topic
        pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)

        publish_pointcloud(pub)
    except rospy.ROSInterruptException:
        pass
    """
    sampling_pcd = Sampling()
    sampling_pcd.start()

