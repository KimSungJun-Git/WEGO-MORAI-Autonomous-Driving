#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan

class Lidar_callback:
    def __init__(self):

        rospy.init_node("lidar_node")
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)
        self.lidar_pub = rospy.Publisher("/lidar_processed", Float64MultiArray, queue_size=10)

        self.lidar_msg = Float64MultiArray()

        
    
    def lidar_callback(self, data):
        rospy.loginfo_once("✅ lidar_callback 실행됨")
        original_ranges = np.array(data.ranges)
        original_ranges[np.isnan(original_ranges)] = 9.9
        remapped_ranges = [0.0] * len(original_ranges)
        for i in range(len(original_ranges)):
            new_index = (i + 180) % 360
            remapped_ranges[new_index] = original_ranges[i]
        self.lidar_msg.data = remapped_ranges
        
        self.lidar_pub.publish(self.lidar_msg)

if __name__ == "__main__":
    try:
        node = Lidar_callback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
