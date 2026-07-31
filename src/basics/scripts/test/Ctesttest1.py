#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from enum import Enum

class State(Enum):
    STRAIGHT = 0
    AVOIDING = 1
    AVOIDING_LEFT = 2
    AVOIDING_RIGHT = 3
    DYNAMIC_STOP = 4
    RETURNING = 5
    STOP = 6        

class Control_node:
    def __init__(self):
        rospy.init_node("controul_node")
        rospy.Subscriber("/lidar_processed", Float64MultiArray, self.control_A)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.servo_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        self.speed_msg = Float64()
        self.servo_msg = Float64()
        self.state = State.STRAIGHT
        self.state_start = rospy.get_time()
        self.target_distance = 2.0
        self.avoid_start = 0.0
        self.avoid_steering_log = []
        self.reduced_log = []
        self.prev_cluster_center = None   # XY좌표(클러스터 중심)
        self.stop_analysis_start = 0.0
        self.stop_analysis_duration = 1.0
        self.stop_ranges_log = []

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    # 1D 클러스터링 (ROI 범위 내 장애물 묶기)
    def find_clusters(self, ranges, start_angle, threshold=0.3, min_points=2):
        clusters = []
        current_cluster = []
        for idx, r in enumerate(ranges):
            angle = start_angle + idx
            if 0.3 < r < 9.8:
                if not current_cluster:
                    current_cluster.append((angle, r))
                else:
                    if abs(r - current_cluster[-1][1]) < threshold:
                        current_cluster.append((angle, r))
                    else:
                        if len(current_cluster) >= min_points:
                            clusters.append(current_cluster)
                        current_cluster = [(angle, r)]
            else:
                if len(current_cluster) >= min_points:
                    clusters.append(current_cluster)
                current_cluster = []
        if len(current_cluster) >= min_points:
            clusters.append(current_cluster)
        return clusters

    def cluster_center_xy(self, cluster):
        # (각도, 거리) → (x, y) 변환 후 평균
        angles, dists = zip(*cluster)
        xs = [r * np.cos(np.deg2rad(a)) for a, r in zip(angles, dists)]
        ys = [r * np.sin(np.deg2rad(a)) for a, r in zip(angles, dists)]
        return np.mean(xs), np.mean(ys)

    def control_A(self, data):
        remapped_ranges = np.array(data.data)
        now = rospy.get_time()
        lane_center_deg = 180
        roi_width = 4
        roi_start = lane_center_deg - roi_width
        roi_end = lane_center_deg + roi_width + 1

        # ROI 거리 데이터
        front_ranges = remapped_ranges[roi_start:roi_end]
        valid_front = [r for r in front_ranges if 0.5 < r < 9.9]
        min_front = min(valid_front) if valid_front else 9.9

        # 클러스터링 (ROI 구간만)
        roi_ranges = remapped_ranges[roi_start:roi_end]
        clusters = self.find_clusters(roi_ranges, roi_start, threshold=0.3, min_points=2)

        # 가장 가까운 클러스터의 중심 xy좌표
        if clusters:
            centers = [np.mean([d for _, d in cluster]) for cluster in clusters]
            nearest_idx = int(np.argmin(centers))
            nearest_cluster = clusters[nearest_idx]
            obstacle_xy = self.cluster_center_xy(nearest_cluster)
        else:
            obstacle_xy = None

        # ========== 상태머신 ==========
        if self.state == State.STRAIGHT:
            self.speed_msg.data = 2000.0
            self.servo_msg.data = 0.5
            if min_front < 2.0:
                self.state = State.STOP
                self.state_start = now
                self.stop_analysis_start = now
                self.stop_ranges_log = []
                self.prev_cluster_center = None
                rospy.loginfo("⏸️ 장애물 감지 → STOP 상태 진입(동/정 분석)")

        elif self.state == State.STOP:
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5
            self.stop_ranges_log.append(remapped_ranges[roi_start:roi_end].copy())

            # 클러스터 중심 위치 변화량
            if obstacle_xy is not None and self.prev_cluster_center is not None:
                dx = obstacle_xy[0] - self.prev_cluster_center[0]
                dy = obstacle_xy[1] - self.prev_cluster_center[1]
                move_dist = np.sqrt(dx**2 + dy**2)
                rospy.loginfo(f"[STOP분석] 장애물 이동: {move_dist:.3f} m")
            else:
                move_dist = 0.0
            self.prev_cluster_center = obstacle_xy

            # 1초 관찰 후 동/정 판별
            if now - self.stop_analysis_start > self.stop_analysis_duration:
                if obstacle_xy is not None and move_dist > 0.25:
                    rospy.loginfo("🚨 동적 장애물로 판단 → DYNAMIC_STOP 상태")
                    self.state = State.DYNAMIC_STOP
                    self.state_start = now
                else:
                    rospy.loginfo("✅ 정적 장애물로 판단 → 회피 진입")
                    self.state = State.AVOIDING
                    self.state_start = now
                    self.avoid_start = now

        elif self.state == State.AVOIDING:
            self.speed_msg.data = 1500.0
            if not hasattr(self, "avoid_count"):
                self.avoid_count = 0
            self.avoid_count += 1   
            if self.avoid_count % 2 == 1:
                self.state = State.AVOIDING_LEFT
                rospy.loginfo("왼쪽 회피 시작")
            else:
                self.state = State.AVOIDING_RIGHT
                rospy.loginfo("오른쪽 회피 시작")
            self.state_start = now
            self.avoid_start = now

        elif self.state == State.AVOIDING_LEFT:
            self.speed_msg.data = 1500.0
            side_ranges = remapped_ranges[90:180]
            valid_side = [r for r in side_ranges if 0.3 < r < 9.9]
            closest_dist = min(valid_side) if valid_side else 9.9
            error = closest_dist - self.target_distance
            steer_adjust = self.sigmoid(error) - 0.5
            self.servo_msg.data = 0.5 + steer_adjust
            self.avoid_steering_log.append(self.servo_msg.data)
            rospy.loginfo(f"[AVOIDING_LEFT] 장애물 거리: {closest_dist:.2f} m | 조향값: {self.servo_msg.data:.2f}")
            if now - self.avoid_start > 0.7:
                self.state = State.RETURNING
                self.state_start = now
                self.reduced_log = self.avoid_steering_log[-7:]
                self.return_index = len(self.reduced_log) - 1
                rospy.loginfo("2차선 회피 종료 → 복귀 시작")

        elif self.state == State.AVOIDING_RIGHT:
            self.speed_msg.data = 1500.0
            side_ranges = remapped_ranges[180:270]
            valid_side = [r for r in side_ranges if 0.3 < r < 9.9]
            closest_dist = min(valid_side) if valid_side else 9.9
            error = closest_dist - self.target_distance
            steer_adjust = self.sigmoid(error) - 0.5
            self.servo_msg.data = 0.5 - steer_adjust
            self.avoid_steering_log.append(self.servo_msg.data)
            rospy.loginfo(f"[AVOIDING_RIGHT] 장애물 거리: {closest_dist:.2f} m | 조향값: {self.servo_msg.data:.2f}")
            if now - self.avoid_start > 0.7:
                self.state = State.RETURNING
                self.state_start = now
                self.reduced_log = self.avoid_steering_log[-7:]
                self.return_index = len(self.reduced_log) - 1
                rospy.loginfo("✅ 1차선 회피 종료 → 복귀 시작")

        elif self.state == State.RETURNING:
            self.speed_msg.data = 1500.0
            if hasattr(self, "return_index") and self.return_index >= 0:
                recovery_val = 0.5 + (0.5 - self.reduced_log[self.return_index])
                self.servo_msg.data = float(np.clip(recovery_val, 0.0, 1.0))
                rospy.loginfo(f"[RETURNING] 복귀 인덱스 {self.return_index} | 조향값: {self.servo_msg.data:.2f}")
                self.return_index -= 1
            else:
                self.servo_msg.data = 0.7
                self.state = State.STRAIGHT
                self.state_start = now
                rospy.loginfo("✅ 2차선 복귀 (STRAIGHT)")

        elif self.state == State.DYNAMIC_STOP:
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5
            if now - self.state_start > 1.0:
                rospy.loginfo("✅ 동적 장애물 사라짐 → STRAIGHT 상태로 복귀")
                self.state = State.STRAIGHT
                self.state_start = now

        self.speed_pub.publish(self.speed_msg)
        self.servo_pub.publish(self.servo_msg)

if __name__ == "__main__":
    try:
        node = Control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
