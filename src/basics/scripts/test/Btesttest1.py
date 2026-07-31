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
        self.previous_ranges = None
        self.last_dynamic_seen_time = 0.0
        self.dynamic_threshold = 0.15
        
        self.avoid_count = 0
        self.current_speed = 1.0
        self.lidar_period = 0.1

        self.dynamic_counter = 0           # 연속 동적 장애물 감지 프레임 수
        self.dynamic_counter_threshold = 3 # 예: 3프레임 연속 필요

        self.prev_obstacle_xy = None  # 장애물 (x, y) 위치 추적용

        # [추가] 장애물 정지 상태에서 분석용 변수
        self.stop_analysis_start = 0.0
        self.stop_analysis_duration = 1.0    # 예: 1초간 관찰
        self.stop_ranges_log = []

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def Dynamic_Obstacle_Detected(self, current_ranges):
        if self.previous_ranges is None:
            self.previous_ranges = current_ranges.copy()
            self.prev_obstacle_xy = None
            return False

        current_segment = np.array(current_ranges[90:270])
        previous_segment = np.array(self.previous_ranges[90:270])
        diffs = np.abs(current_segment - previous_segment)
        moving_score = np.mean(diffs)
        my_speed = self.current_speed
        lidar_period = self.lidar_period
        expected_change = my_speed * lidar_period
        corrected_score = max(0, moving_score - expected_change)
        valid_distances = [r for r in current_segment if 1.0 < r < 9.9]
        min_distance = min(valid_distances) if valid_distances else 9.9
        min_idx = np.argmin(current_segment)
        min_angle = 150 + min_idx
        obstacle_xy = self.location_xy(current_segment[min_idx], min_angle)
        dist_change = 0
        if self.prev_obstacle_xy is not None:
            dx = obstacle_xy[0] - self.prev_obstacle_xy[0]
            dy = obstacle_xy[1] - self.prev_obstacle_xy[1]
            dist_change = np.sqrt(dx ** 2 + dy ** 2)
        self.prev_obstacle_xy = obstacle_xy
        self.previous_ranges = current_ranges.copy()
        if corrected_score > self.dynamic_threshold and min_distance < 4.0 and dist_change > 0.3:
            self.last_dynamic_seen_time = rospy.get_time()
            return True
        return False

    def location_xy(self, r, theta_deg):
        theta = np.deg2rad(theta_deg)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    def control_A(self, data):
        rospy.loginfo_once("✅ control_A 실행됨")

        # ROI
        remapped_ranges = np.array(data.data)
        lane_center_deg = 180
        roi_width = 3
        roi_start = lane_center_deg - roi_width
        roi_end = lane_center_deg + roi_width + 1
        front_ranges = remapped_ranges[roi_start:roi_end]
        valid_front = [r for r in front_ranges if 0.5 < r < 9.9]
        min_front = min(valid_front) if valid_front else 9.9
        now = rospy.get_time()

        if valid_front:
            min_idx = np.argmin(front_ranges)
            min_dist = front_ranges[min_idx]
            min_angle = roi_start + min_idx
            x, y = self.location_xy(min_dist, min_angle)

        # ========== STRAIGHT ==========
        if self.state == State.STRAIGHT:
            self.speed_msg.data = 2000.0
            self.servo_msg.data = 0.5
            if min_front < 2.0:
                # 정면 장애물 감지 시, STOP 상태로 진입해서 분석 시작!
                self.state = State.STOP
                self.state_start = now
                self.stop_analysis_start = now
                self.stop_ranges_log = []
                rospy.loginfo("⏸️ 장애물 감지 → STOP 상태 진입(동/정 분석)")
        
        # ========== STOP: 정지 후 장애물 분석 ==========
        # STOP 상태에서 관찰할 때, 모든 프레임 변화량을 기록!
        elif self.state == State.STOP:
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5
            
            self.stop_ranges_log.append(remapped_ranges[90:270].copy())
            # 관찰 시간 경과 시, 변화량으로 동/정적 판별
            if now - self.stop_analysis_start > self.stop_analysis_duration:
                if len(self.stop_ranges_log) > 0.5: #1초동안
                    start_ranges = self.stop_ranges_log[0]
                    end_ranges = self.stop_ranges_log[-1]
                    diffs = np.abs(np.array(end_ranges) - np.array(start_ranges))
                    moving_score = np.mean(diffs)
                else:
                    moving_score = 0.0

                rospy.loginfo(f"[STOP분석] moving_score={moving_score:.4f}")
                if moving_score > 0.2:    # threshold는 센서환경에 따라 조정
                    rospy.loginfo("🚨 동적 장애물로 판단 → DYNAMIC_STOP 상태")
                    self.state = State.DYNAMIC_STOP
                    self.state_start = now
                    self.last_dynamic_seen_time = now
                else:
                    rospy.loginfo("✅ 정적 장애물로 판단 → 회피 진입")
                    self.state = State.AVOIDING
                    self.state_start = now
                    self.avoid_start = now



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
