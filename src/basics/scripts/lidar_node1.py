#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan

class LidarCallback:
    def __init__(self):
        rospy.init_node("lidar_node")
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_callback)
        self.lidar_pub = rospy.Publisher("/lidar_processed", Float64MultiArray, queue_size=10)
        self.lidar_msg = Float64MultiArray()

    def find_clusters(self, ranges, threshold=0.3, min_points=3):
        clusters = []
        current_cluster = []
        for i, r in enumerate(ranges):
            if r < 9.8:  # 실거리 (최대값/노이즈 제외)
                if not current_cluster:
                    current_cluster.append((i, r))
                else:
                    if abs(r - current_cluster[-1][1]) < threshold:
                        current_cluster.append((i, r))
                    else:
                        if len(current_cluster) >= min_points:
                            clusters.append(current_cluster)
                        current_cluster = [(i, r)]
            else:
                if len(current_cluster) >= min_points:
                    clusters.append(current_cluster)
                current_cluster = []
        if len(current_cluster) >= min_points:
            clusters.append(current_cluster)
        return clusters

    def lidar_callback(self, data):
        rospy.loginfo_once("✅ lidar_callback 실행됨")
        
        # raw ranges
        original_ranges = np.array(data.ranges)
        invalid_mask = np.isnan(original_ranges) | np.isinf(original_ranges)
        original_ranges[invalid_mask] = 9.9

        n = len(original_ranges)
        remapped_ranges = np.roll(original_ranges, 180 if n == 360 else n // 2)

        # 값 요약
        center_idx = n // 2
        front_val = remapped_ranges[center_idx]
        min_val = np.min(remapped_ranges)
        max_val = np.max(remapped_ranges)
        mean_val = np.mean(remapped_ranges)
        roi_width = 5
        roi = remapped_ranges[center_idx - roi_width:center_idx + roi_width + 1]
        roi_mean = np.mean(roi)

        rospy.loginfo(f"[LIDAR] 정면 거리: {front_val:.2f} m | min: {min_val:.2f} | max: {max_val:.2f} | mean: {mean_val:.2f} | ROI(±5도) mean: {roi_mean:.2f}")

        # 클러스터링
        clusters = self.find_clusters(remapped_ranges, threshold=0.3, min_points=3)
        rospy.loginfo(f"[LIDAR] 감지된 장애물(클러스터) 수: {len(clusters)}")
        for idx, cluster in enumerate(clusters):
            indices, distances = zip(*cluster)
            center_angle = np.mean(indices)
            center_distance = np.mean(distances)
            size = len(cluster)
            rospy.loginfo(f"  ▶️ #{idx+1} : 중심각={center_angle:.1f}°, 거리={center_distance:.2f}m, 포인트수={size}")

        # (필요시) 부분값 디버그
        rospy.logdebug(f"Remapped ranges[175:185]: {remapped_ranges[center_idx-5:center_idx+6]}")

        self.lidar_msg.data = remapped_ranges.tolist()
        self.lidar_pub.publish(self.lidar_msg)

if __name__ == "__main__":
    try:
        node = LidarCallback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
