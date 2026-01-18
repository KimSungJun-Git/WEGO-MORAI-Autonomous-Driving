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
        self.previous_ranges = None
        self.last_dynamic_seen_time = 0.0
        self.dynamic_threshold = 0.5
        self.AVOIDING_DURATION = 0.7
        self.avoid_count = 0
        self.current_speed = 1.0
        self.lidar_period = 0.1

        self.dynamic_counter = 0           # 연속 동적 장애물 감지 프레임 수
        self.dynamic_counter_threshold = 3 # 예: 3프레임 연속 필요


    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def Dynamic_Obstacle_Detected(self, current_ranges):
        if self.previous_ranges is None:
            self.previous_ranges = current_ranges.copy()
            return False
        if corrected_score > self.dynamic_threshold and min_distance < 2.0:
            self.dynamic_counter += 1
        else:
            self.dynamic_counter = 0
        current_segment = np.array(current_ranges[150:210])
        previous_segment = np.array(self.previous_ranges[150:210])
        diffs = np.abs(current_segment - previous_segment)
        moving_score = np.mean(diffs)
        my_speed = self.current_speed
        lidar_period = self.lidar_period
        expected_change = my_speed * lidar_period
        corrected_score = max(0, moving_score - expected_change)
        self.previous_ranges = current_ranges.copy()
        valid_distances = [r for r in current_segment if 1.0 < r < 9.9]
        min_distance = min(valid_distances) if valid_distances else 9.9
        if corrected_score > self.dynamic_threshold and min_distance < 2.0:
            self.last_dynamic_seen_time = rospy.get_time()
            rospy.loginfo(f"🚨 동적 장애물 감지: 이동점수={corrected_score:.2f}, 최소거리={min_distance:.2f}m")
            return True
        return False



    def control_A(self, data):
        rospy.loginfo_once("✅ control_A 실행됨")
        remapped_ranges = np.array(data.data)
        lane_center_deg = 180
        roi_width = 3
        roi_start = lane_center_deg - roi_width
        roi_end = lane_center_deg + roi_width + 1
        front_ranges = remapped_ranges[roi_start:roi_end]
        valid_front = [r for r in front_ranges if 0.5 < r < 9.9]
        min_front = min(valid_front) if valid_front else 9.9
        now = rospy.get_time()

        # 🔥 STOP 없이 바로 동적/정적 판단 후 상태 전이
        if self.state == State.STRAIGHT:
            self.speed_msg.data = 2000.0
            self.servo_msg.data = 0.5
            if min_front < self.target_distance:
                # 장애물 감지 시 바로 동적 장애물 판별
                if self.Dynamic_Obstacle_Detected(remapped_ranges):
                    self.state = State.DYNAMIC_STOP
                    self.state_start = now
                    rospy.loginfo("🚨 동적 장애물 감지 → 정지 유지")
                else:
                    self.state = State.AVOIDING
                    self.state_start = now
                    self.avoid_start = now
                    rospy.loginfo("✅ 정적 장애물 → 회피 진입")

        elif self.state == State.AVOIDING:
            self.speed_msg.data = 1500.0
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
            if self.return_index >= 0:
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
            if now - self.last_dynamic_seen_time > 1.0:
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
