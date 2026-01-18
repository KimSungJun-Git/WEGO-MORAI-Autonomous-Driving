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

        # 장애물 회피 변수
        self.state = State.STRAIGHT
        self.state_start = rospy.get_time()
        self.target_distance = 2.0
        self.avoid_start = 0.0
        self.avoid_steering_log = []
        self.reduced_log = []
        # 동적 장애물 변수
        self.previous_ranges = None
        self.stop_start_time = 0.0
        self.last_dynamic_seen_time = 0.0
        self.dynamic_threshold = 0.4 #  이걸 1.0으로 바꾸면 1차선 주행시 2차선에 있는 박스를 인식 안함, 자전거도 인식 불가 차량은 2.0~3.0정도
        #self.dynamic_detection_angle_range = (150, 210)
        self.AVOIDING_DURATION = 0.7
        self.avoid_count = 0

        self.current_speed = 1.0  # 임시값
        self.lidar_period = 0.1   # 임시값

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def Dynamic_Obstacle_Detected(self, current_ranges):
        if self.previous_ranges is None:
            self.previous_ranges = current_ranges.copy()
            return False

        current_segment = np.array(current_ranges[150:210])
        previous_segment = np.array(self.previous_ranges[150:210])

        # 변화량 계산
        diffs = np.abs(current_segment - previous_segment)
        moving_score = np.mean(diffs)

        # 차 이동에 의한 변화량 보정
        my_speed = self.current_speed  # m/s 단위 (예: self.current_speed 멤버 변수로 받아오기)
        lidar_period = self.lidar_period  # LiDAR 주기 (초 단위, 예: 0.1 등)
        expected_change = my_speed * lidar_period
        corrected_score = moving_score - expected_change
        # 음수 방지
        corrected_score = max(0, corrected_score)

        self.previous_ranges = current_ranges.copy()

        # 최소 거리 계산
        valid_distances = [r for r in current_segment if 1.0 < r < 9.9]
        min_distance = min(valid_distances) if valid_distances else 9.9

        # 거리 + 움직임 동시 판단
        if corrected_score > self.dynamic_threshold and min_distance < 2.0:
            self.last_dynamic_seen_time = rospy.get_time()
            rospy.loginfo(f"🚨 동적 장애물 감지: 이동점수={corrected_score:.2f}, 최소거리={min_distance:.2f}m")
            return True
        return False


    def control_A(self, data):
        rospy.loginfo_once("✅ control_A 실행됨")
        remapped_ranges = np.array(data.data)

        lane_center_deg = 180  # 내 차선(주행 경로) 중심 각도
        roi_width = 3          # ±3도 (총 7도)
        roi_start = lane_center_deg - roi_width    # 177
        roi_end = lane_center_deg + roi_width + 1  # 184 (파이썬 인덱스 슬라이스 특성상 +1)

        front_ranges = remapped_ranges[roi_start:roi_end]  # [177,178,179,180,181,182,183]
        valid_front = [r for r in front_ranges if 0.5 < r < 9.9]
        min_front = min(valid_front) if valid_front else 9.9

    # 전방 장애물 탐지 (최소거리만)

        now = rospy.get_time()
        # 상태별 제어
        if self.state == State.STRAIGHT:
            self.speed_msg.data = 2000.0
            self.servo_msg.data = 0.5
            if min_front < self.target_distance:
                self.state = State.STOP
                self.state_start = now
                rospy.loginfo("⚠️ STOP: 장애물 감지, 일시 정지 후 분석 시작")

        elif self.state == State.STOP:
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5
            # 정지 후 0.5초 정도 관찰
            if now - self.state_start > 1:
                # 동적 장애물 판별
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
            self.servo_msg.data = 0.5 + steer_adjust ###
            self.avoid_steering_log.append(self.servo_msg.data)
            rospy.loginfo(f"[AVOIDING_RIGHT] 장애물 거리: {closest_dist:.2f} m | 조향값: {self.servo_msg.data:.2f}")
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
            self.servo_msg.data = 0.5 - steer_adjust###
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

