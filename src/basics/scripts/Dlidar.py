#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Int32
from sensor_msgs.msg import LaserScan

class Lidar_callback:
    def __init__(self):
        rospy.init_node("lidar_node")  
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)  
        
        self.lidar_pub = rospy.Publisher("/lidar_processed", Float64MultiArray, queue_size=10)
        self.cluster_pub = rospy.Publisher("/lidar_clusters", Float64MultiArray, queue_size=10)
        self.cluster_count_pub = rospy.Publisher("/lidar_cluster_count", Int32, queue_size=10)

        self.lidar_msg = Float64MultiArray()
        self.cluster_msg = Float64MultiArray()

    def lidar_callback(self, data):
        rospy.loginfo_once("✅ lidar_callback 실행됨")
        original_ranges = np.array(data.ranges)
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        
        # NaN 처리
        original_ranges[np.isnan(original_ranges)] = 5.0
        
        # 0~359도 원본 데이터를 180도 기준 좌우 반전시켜 재배열
        remapped_ranges = [0.0] * len(original_ranges)
        for i in range(len(original_ranges)):
            new_index = (i + 180) % len(original_ranges)
            remapped_ranges[new_index] = original_ranges[i]

        # remapped 데이터 발행
        self.lidar_msg.data = remapped_ranges
        self.lidar_pub.publish(self.lidar_msg)

        points = []
        for i, r in enumerate(original_ranges):
            if r < 5.0:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
        points = np.array(points)

        def euclidean_clustering(points, eps=0.5, min_samples=3):
            clusters = []
            used = set()
            for i, p in enumerate(points):
                if i in used:
                    continue
                cluster = [i]
                queue = [i]
                used.add(i)
                while queue:
                    idx = queue.pop(0)
                    for j, q in enumerate(points):
                        if j in used:
                            continue
                        if np.linalg.norm(points[idx] - q) < eps:
                            cluster.append(j)
                            queue.append(j)
                            used.add(j)
                if len(cluster) >= min_samples:
                    clusters.append(cluster)
            return clusters

        cluster_centers = [] #중심 계산
        if len(points) > 0:
            clusters = euclidean_clustering(points, eps=0.5, min_samples=3)
            for cluster in clusters:
                cluster_points = points[cluster]
                center = np.mean(cluster_points, axis=0)
                cluster_centers.extend(center.tolist())

        self.cluster_msg.data = cluster_centers
        self.cluster_pub.publish(self.cluster_msg)
        self.cluster_count_pub.publish(Int32(len(cluster_centers)//2))


if __name__ == "__main__":
    try:
        node = Lidar_callback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
