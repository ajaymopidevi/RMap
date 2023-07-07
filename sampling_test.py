#!/usr/bin/env python3

import rospy
import struct
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def publish_pointcloud():
    # Initialize ROS node
    rospy.init_node('pointcloud_publisher', anonymous=True)
    
    # Create a publisher for the PointCloud2 topic
    pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=10)
    
    # Create the PointCloud2 message
    cloud_msg = PointCloud2()
    
    # Set the header
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = 'map'
    
    # Set the fields (x, y, z, intensity)
    cloud_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.UINT32, count=1),
    ]
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16
    cloud_msg.row_step = cloud_msg.point_step * 4
    
    # Set the data
    points = [
        (0.0, 0.0, 0.0, 1),
        (1.0, 0.0, 0.0, 2),
        (0.0, 1.0, 0.0, 3),
        (1.0, 1.0, 0.0, 4)
    ]
    cloud_msg.width = len(points)
    cloud_msg.height = 1
    cloud_msg.data = []
    for point in points:
        cloud_msg.data.extend(struct.pack('<fffI', *point))
    
    # Publish the PointCloud2 message repeatedly
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()
        pub.publish(cloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pointcloud()
    except rospy.ROSInterruptException:
        pass
