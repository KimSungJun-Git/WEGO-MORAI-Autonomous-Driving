#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from math import *

class LidarReceiver: 
    def __init__(self):
        rospy.init_node("lidarReceiver")
        rospy.Subscriber("/lidar2D", LaserScan, self.LidarCB)

        self.angle = []  # LiDAR에서 받은 최신 데이터를 저장

    def LidarCB(self, data):
        self.angle = data

        # 원래 거리값 가져오기
        original_ranges = list(data.ranges)

        # 중심 정렬된 거리값 배열 생성
        remapped_ranges = [0.0] * len(original_ranges)
        for i in range(len(original_ranges)):
            new_index = (i + 180) % 360
            remapped_ranges[new_index] = original_ranges[i]

        # 인덱스, 각도(도), 거리값(미터) 출력
        indexed_output = [
            f"[{idx:03d}] ({idx:>3}°): {r:>5.2f}m"
            for idx, r in enumerate(remapped_ranges)
        ]

        print("중심 정렬된 거리 데이터 (각도 포함):")
        print("  ".join(indexed_output))  

if __name__ == "__main__":
    try:
        Lr = LidarReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
