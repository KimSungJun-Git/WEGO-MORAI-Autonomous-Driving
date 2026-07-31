import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Int32
from enum import Enum

class State(Enum):
    STRAIGHT = 0
    AVOIDING = 1
    AVOIDING_LEFT = 2
    AVOIDING_RIGHT = 3
    DYNAMIC_STOP = 4
    RETURNING = 5
    STOP = 6    
    WAIT_ROUNDABOUT = 7    
    CHECK_GAP = 8
    ENTER_ROUNDABOUT = 9
    EXIT_ROUNDABOUT = 10
    

class Control_node:
    def __init__(self):
        rospy.init_node("controul_node")
        rospy.Subscriber("/lidar_processed", Float64MultiArray, self.control_A)
        rospy.Subscriber("/lidar_clusters", Float64MultiArray, self.clusters_callback)
        rospy.Subscriber("/lidar_cluster_count", Int32, self.cluster_count_callback)
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
        self.dynamic_counter = 0 # 연속 동적 장애물 감지 프레임 수
        self.dynamic_counter_threshold = 3  # 예: 3프레임 연속 필요
        self.prev_obstacle_xy = None # 장애물 (x, y) 위치 추적용
        

        # [추가] 장애물 정지 상태에서 분석용 변수
        self.stop_analysis_start = 0.0
        self.stop_analysis_duration = 1.0 #1초간 관찰
        self.stop_ranges_log = []
        
        # 클러스터 관련
        self.current_clusters = np.empty((0, 2))
        self.prev_clusters = None
        self.cluster_count = 0
        # Gap & 회전판단 관련
        self.move_threshold = 0.15
        self.roundabout_min = 2
        self.roundabout_max = 3
        
        self.roundabout_detect_count = 0
        self.roundabout_detect_needed = 3  # 3프레임 연속 감지 시 전이(오탐 방지)

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def Dynamic_Obstacle_Detected(self, current_ranges):
        current_ranges = np.array(current_ranges)
        if self.previous_ranges is None:
            self.previous_ranges = np.copy(current_ranges)
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
        valid_distances = [r for r in current_segment if 1.0 < r < 5.0]
        min_distance = min(valid_distances) if valid_distances else 5.0
        min_idx = np.argmin(current_segment)
        min_angle = 150 + min_idx
        obstacle_xy = self.location_xy(current_segment[min_idx], min_angle)
        dist_change = 0
        if self.prev_obstacle_xy is not None:
            dx = obstacle_xy[0] - self.prev_obstacle_xy[0]
            dy = obstacle_xy[1] - self.prev_obstacle_xy[1]
            dist_change = np.sqrt(dx ** 2 + dy ** 2)
        self.prev_obstacle_xy = obstacle_xy
        self.previous_ranges = np.copy(current_ranges)
        if corrected_score > self.dynamic_threshold and min_distance < 4.0 and dist_change > 0.3:
            self.last_dynamic_seen_time = rospy.get_time()
            return True
        return False

    def location_xy(self, r, theta_deg):
        theta = np.deg2rad(theta_deg)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    def clusters_callback(self, msg):
        
        if not msg.data:
            self.current_clusters = np.empty((0, 2))
        else:
            clusters = np.array(msg.data).reshape(-1, 2)
            self.current_clusters = clusters
        
    def cluster_count_callback(self, msg):
        self.cluster_count = msg.data  # 현재 감지된 장애물 개수 저장

    def Dynamic_Obstacle_roundabout(self, move_threshold=0.15, count_min=2, count_max=5):
        if self.prev_clusters is None or len(self.prev_clusters) == 0:
            self.prev_clusters = self.current_clusters
            return False
        move_count = 0
        for curr in self.current_clusters:
            # 이전 프레임에서 가장 가까운 클러스터 찾아 거리 비교
            dists = [np.linalg.norm(curr - prev) for prev in self.prev_clusters]
            if dists and min(dists) > move_threshold:
                move_count += 1  # 움직임 있음
    
        self.prev_clusters = self.current_clusters
        # 움직이는 클러스터(차량)가 2~3대면 돌고 있다
        return count_min <= move_count <= count_max
    
    #Gap틈 체크 함수
    def check_gap_around(self, clusters, min_gap=2.0, center_angle=225, width=40):
        """
        지정 각도 범위 내에 gap이 있는지, gap의 위치(중앙 좌표)까지 반환
        Returns:
            gap_exists (bool), gap_position (np.array or None)
        """
        # 1. 각도 계산
        cluster_angles = [np.rad2deg(np.arctan2(y, x)) % 360 for x, y in clusters]
        lower = (center_angle - width/2) % 360
        upper = (center_angle + width/2) % 360

        # 2. 각도 범위 내 클러스터 추출
        def in_range(angle, lower, upper):
            if lower < upper:
                return lower <= angle <= upper
            else:
                return angle >= lower or angle <= upper

        in_direction_clusters = []
        for idx, angle in enumerate(cluster_angles):
            if in_range(angle, lower, upper):
                in_direction_clusters.append(clusters[idx])

        # 3. 클러스터가 없으면 gap은 무한히 넓음 → True, (0,0) 반환
        if not in_direction_clusters:
            return True, np.array([0.0, 0.0])

        # 4. 두 클러스터 사이 gap 검사 + 가장 넓은 gap 좌표 찾기
        max_gap = 0
        gap_position = None
        N = len(in_direction_clusters)
        if N == 1:
            # 한 대만 있으면 차량 좌표 기준 약간 바깥 방향으로 진입
            single_cluster = np.array(in_direction_clusters[0])
            gap_position = single_cluster / np.linalg.norm(single_cluster) * (min_gap + 0.5)
            return True, gap_position

        for i in range(N):
            for j in range(i+1, N):
                pt1 = np.array(in_direction_clusters[i])
                pt2 = np.array(in_direction_clusters[j])
                dist = np.linalg.norm(pt1 - pt2)
                if dist > max_gap and dist > min_gap:
                    max_gap = dist
                    gap_position = (pt1 + pt2) / 2  # gap 중앙

        if gap_position is not None:
            return True, gap_position

        # gap 없음
        return False, None

    def get_lead_vehicle_xy(self):
        """
        180~270도 각도 범위 내에서 가장 가까운 클러스터(차량)의 (x, y) 좌표 반환
        (없으면 None)
        """
        if not hasattr(self, "current_clusters") or len(self.current_clusters) == 0:
            return None
        min_r = float("inf")
        lead_xy = None
        for x, y in self.current_clusters:
            angle = np.rad2deg(np.arctan2(y, x)) % 360
            if 90 <= angle <= 180:
                r = np.sqrt(x**2 + y**2)
                if r < min_r:
                    min_r = r
                    lead_xy = (x, y)
        return lead_xy

    def get_dynamic_lead_vehicle_xy(self, prev_clusters, move_thresh=0.1):
        if prev_clusters is None or len(prev_clusters) == 0 or len(prev_clusters) != len(self.current_clusters):
            return None
        candidates = []
        for idx, (x, y) in enumerate(self.current_clusters):
            angle = np.rad2deg(np.arctan2(y, x)) % 360
            if 180 <= angle <= 270:
                px, py = prev_clusters[idx]
                move_dist = np.linalg.norm([x - px, y - py])
                if move_dist > move_thresh:
                    r = np.sqrt(x**2 + y**2)
                    candidates.append((r, (x, y)))
        if candidates:
            return min(candidates, key=lambda c: c[0])[1]
        return None


    def get_lead_vehicle_speed(self):
        # 앞차 좌표 변화량 기반 (프레임당 이동 거리)
        if self.prev_clusters is None or len(self.prev_clusters) == 0 or len(self.current_clusters) == 0:
            return 0.0
        lead_xy = self.get_lead_vehicle_xy()
        if lead_xy is None:
            return 0.0
        prev_lead_xy = min(self.prev_clusters, key=lambda xy: np.linalg.norm(np.array(xy) - np.array(lead_xy)))
        move_dist = np.linalg.norm(np.array(lead_xy) - np.array(prev_lead_xy))
        speed = move_dist / self.lidar_period
        return speed

    def get_lead_vehicle_steer(self):
        lead_xy = self.get_lead_vehicle_xy()
        if lead_xy is None:
            return 0.0
        x, y = lead_xy
        angle = np.arctan2(y, x)
        return angle

    def steer_to(self, target_xy):
        x, y = target_xy
        angle = np.arctan2(y, x)  # 라디안
        steer = 0.5 + np.clip(angle/np.pi, -0.3, 0.3)
        self.servo_msg.data = float(np.clip(steer, 0.0, 1.0))

    def is_exit_point_reached(self):
        # 탈출 조건(예: 특정 각도, 거리, 주행 시간 등)
        # 실제 상황에 맞게 보완 필요
        # 여기선 예시로 3초간 주행하면 탈출로 간주
        now = rospy.get_time()
        return (now - self.state_start) > 6.0 ###############3

    def has_exited_roundabout(self):
        # 탈출 완료 조건 (예시: 조향 1.0 상태 1초 유지 등)
        # 실제 상황에 맞게 보완 필요
        now = rospy.get_time()
        return (now - self.state_start) > 1.0

        
    def control_A(self, data):
        rospy.loginfo_once("✅ control_A 실행됨")

        # ROI
        remapped_ranges = np.array(data.data)
        lane_center_deg = 180
        roi_width = 3
        roi_start = lane_center_deg - roi_width
        roi_end = lane_center_deg + roi_width + 1
        front_ranges = remapped_ranges[roi_start:roi_end]
        valid_front = [r for r in front_ranges if 0.5 < r < 5.0]
        min_front = min(valid_front) if valid_front else 5.0
        now = rospy.get_time()

        if valid_front:
            min_idx = np.argmin(front_ranges)
            min_dist = front_ranges[min_idx]
            min_angle = roi_start + min_idx
            x, y = self.location_xy(min_dist, min_angle)
            print(f"x: {x:.2f}, y: {y:.2f}, r: {min_dist:.2f}, angle: {min_angle:.2f}")
            rospy.loginfo(f"x: {x:.2f}, y: {y:.2f}, r: {min_dist:.2f}, angle: {min_angle:.2f}")
        

        #147:167
        #왼쪽이 270
        # ========== STRAIGHT ==========
        if self.state == State.STRAIGHT:
            self.speed_msg.data = 2000.0
            self.servo_msg.data = 0.5

            # 클러스터가 있을 때만 각도 계산 및 라운드어바웃 접근 판별
            if len(self.current_clusters) > 0:
                # 거리와 각도 모두 만족하는 클러스터만 카운트
                in_range_count = 0
                for x, y in self.current_clusters:
                    r = np.sqrt(x**2 + y**2)
                    angle = np.rad2deg(np.arctan2(y, x)) % 360
                    print(f"클러스터 x: {x:.2f}, y: {y:.2f}, r: {r:.2f}, angle: {angle:.2f}")

                    if 150 <= angle <= 270 and 1.0< r < 4.0: #4m 이하 ##############################
                        in_range_count += 1
                        

                # 2~4개 감지되고, 3프레임 연속이면 WAIT_ROUNDABOUT로 전이
                if 2 <= in_range_count <= 4:
                    self.roundabout_detect_count += 1
                    if self.roundabout_detect_count >= 3:  # 3프레임 연속 감지
                        rospy.loginfo("로터리 접근 감지! WAIT_ROUNDABOUT 상태로 전이")
                        #self.speed_msg.data = 0.0
                        
                        self.state = State.WAIT_ROUNDABOUT
                        self.state_start = now
                        self.roundabout_detect_count = 0  # 초기화
                else:
                    self.roundabout_detect_count = 0  # 조건 미달시 초기화
            else:
                self.roundabout_detect_count = 0  # 클러스터 없으면 초기화

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
            valid_side = [r for r in side_ranges if 0.3 < r < 5.0]
            closest_dist = min(valid_side) if valid_side else 5.0
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
            valid_side = [r for r in side_ranges if 0.3 < r < 5.0]
            closest_dist = min(valid_side) if valid_side else 5.0
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

        elif self.state == State.WAIT_ROUNDABOUT:
            self.speed_msg.data = 0.0  # 일단 정지
            gap_ok, _ = self.check_gap_around(self.current_clusters, min_gap=2.0, center_angle=160, width=40)
            if gap_ok:
                self.state = State.CHECK_GAP
                self.state_start = now

        elif self.state == State.CHECK_GAP:
            gap_ok, target_vehicle = self.check_gap_around(self.current_clusters, min_gap=2.0, center_angle=160, width=40)
            if gap_ok and target_vehicle is not None:
                self.state = State.ENTER_ROUNDABOUT
                self.target_follow_xy = target_vehicle
                self.state_start = now


        elif self.state == State.ENTER_ROUNDABOUT:
            lead_xy = self.get_dynamic_lead_vehicle_xy(self.prev_clusters, move_thresh=0.1)
            if lead_xy is None:
                lead_xy = self.target_follow_xy      # 앞차 좌표
            lead_speed = self.get_lead_vehicle_speed() # 앞차 속도 (거리 변화량 등)
            lead_steer = self.get_lead_vehicle_steer() # 앞차 조향 (각도 변화량 등)

            # 2. 내 차량 steering/speed를 앞차에 맞춤
            if lead_xy is not None:
                self.steer_to(lead_xy)
                self.speed_msg.data = max(1100, lead_speed * 0.95)

            if self.is_exit_point_reached():
                self.state = State.EXIT_ROUNDABOUT
                self.state_start = now

        elif self.state == State.EXIT_ROUNDABOUT:
            # roundabout 탈출 판단, 직진 복귀 등
            if self.has_exited_roundabout():
                # 우회전 해서 탈출 
                self.servo_msg.data = 0.7 # 조향
                self.speed_msg.data = 1000 # 속도
                # 탈출 완료 시 STRAIGHT 등으로 전이
                if self.has_exited_roundabout():
                    self.state = State.STRAIGHT
                    self.state_start = now
        

        self.speed_pub.publish(self.speed_msg)
        self.servo_pub.publish(self.servo_msg)
        self.prev_clusters = np.copy(self.current_clusters)

if __name__ == "__main__":
    try:
        node = Control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
