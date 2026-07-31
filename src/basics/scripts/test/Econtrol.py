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
    END = 11
    

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

        self.entry_xy = None   # 로터리 진입시 차량 위치

        # === ACC(앞차 추종) 파라미터 ===
        self.v_cmd_min = 300      # 모터 최소 명령
        self.v_cmd_max = 1200     # 모터 최대 명령 (라운드어바웃 내부 과속 방지) 1200 ~ 1500
        self.v_cmd_cruise = 1000  # 리더 없을 때 기본 순항 속도

        self.s0 = 0.6             # [m] 최소 정지 간격
        self.T_headway = 1.2      # [s] 헤드웨이                    충돌 회피 민감도 1.2로 시작 → 붙는 느낌 있으면 1.3~1.5로 단계적으로 키움.
        self.k_gap = 800.0        # [cmd/m] 간격 오차 게인
        self.k_rel = 500.0        # [cmd/(m/s)] 상대속도 게인       감속 세기 접근 시 감속

        self.ttc_thresh = 1.0     # [s] TTC 임계 시간-대-충돌(Time To Collision)**이 이 값보다 작아지면 강한 감속/정지. 1.0 ~ 1.5 s 
        self.max_delta_cmd = 200  # [cmd/step] 가감속 제한(저크 억제) 50 ~ 300 
        self.v_cmd_prev = 0

        # 내부 상태
        self.prev_lead_dist = None
        self.lead_speed_lpf = 0.0
        self.lpf_alpha = 0.6      # 0~1, 클수록 반응 빠름

        # === Follow duration scaling (거리 보존) ===
        self.follow_t_base = 2.5     # 기준 시간(초) : 기존 2.5초
        self.follow_v_base = 800.0   # 기준 속도(cmd) : “800” 기준
        self.v_cmd_eps = 1.0         # 0 나눗셈 방지

        # === ACC(앞차 추종) 파라미터 ===
        self.v_cmd_min = 300
        self.v_cmd_max = 1500
        self.v_cmd_cruise = 1000

        self.s0 = 0.6           # [m]
        self.T_headway = 1.2    # [s] (필요시 1.5까지)
        self.ttc_thresh = 1.2   # [s] (필요시 1.0~1.2)

        self.max_delta_cmd = 200
        self.v_cmd_prev = 0
        self.prev_lead_dist = None
        self.lead_speed_lpf = 0.0
        self.lpf_alpha = 0.6
        # === Follow duration scaling ===
        self.follow_t_base = 2.5
        self.follow_v_base = 800.0
        self.v_cmd_eps = 1.0
        
        # === Sigmoid 셰이핑 ===
        self.g_scale = 0.5
        self.v_scale = 0.5
        self.w_gap = 0.6
        self.w_rel = 0.4
        self.decel_cmd_max = 600

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
        arr = np.array(msg.data, dtype=float)
        n = (arr.size // 2) * 2
        if n == 0:
            self.current_clusters = np.empty((0, 2))
        else:
            self.current_clusters = arr[:n].reshape(-1, 2)
        
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
    def check_gap_around(self, clusters, min_gap=1.0, center_angle=225, width=40):
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
        # 최근 2프레임 사이에 "실제로 움직이는" 클러스터를 찾음
        if prev_clusters is None or len(prev_clusters) == 0 or len(prev_clusters) != len(self.current_clusters):
            return None
        candidates = []
        for idx, (x, y) in enumerate(self.current_clusters):
            angle = np.rad2deg(np.arctan2(y, x)) % 360
            if 160 <= angle <= 270:  # 로터리 왼쪽~뒤쪽(진입~진출 각도 범위)
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
        steer = 0.5 - np.clip(angle/np.pi, -0.3, 0.3)
        self.servo_msg.data = float(np.clip(steer, 0.0, 1.0))

    def is_exit_point_reached(self):
        # 탈출 조건(예: 특정 각도, 거리, 주행 시간 등)
        # 실제 상황에 맞게 보완 필요
        # 여기선 예시로 3초간 주행하면 탈출로 간주
        now = rospy.get_time()
        return (now - self.state_start) > 2.5 #############################################################

    def has_exited_roundabout(self):
        # 탈출 완료 조건 (예시: 조향 1.0 상태 1초 유지 등)
        # 실제 상황에 맞게 보완 필요
        now = rospy.get_time()
        return (now - self.state_start) > 1.0

        #90~180도 범위 내 클러스터 유무 체크 함수

    def exist_cluster_in_angle(self, clusters, angle_min=90, angle_max=160):
        for x, y in clusters:
            angle = np.rad2deg(np.arctan2(y, x)) % 360
            if angle_min <= angle <= angle_max:
                return True
        return False
#앞차 따라가기 속도 비례
    def _clamp(self, x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def _lpf(self, prev, new, alpha):
        return alpha*new + (1.0-alpha)*prev

    def compute_follow_time(self, v_now):
        v = max(float(v_now), self.v_cmd_eps)
        # 거리 보존: t_need = t_base * (v_base / v_now)
        return self.follow_t_base * (self.follow_v_base / v)
    

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
            #print(f"x: {x:.2f}, y: {y:.2f}, r: {min_dist:.2f}, angle: {min_angle:.2f}")
            #rospy.loginfo(f"x: {x:.2f}, y: {y:.2f}, r: {min_dist:.2f}, angle: {min_angle:.2f}")q
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
                    if 150 <= angle <= 270 and 0.5 < r < 4.0: #4m 이하 ################################################
                        in_range_count += 1

                # 2~4개 감지되고, 3프레임 연속이면 WAIT_ROUNDABOUT로 전이
                if 2 <= in_range_count <= 5:
                    self.roundabout_detect_count += 1
                    if self.roundabout_detect_count >= 2:  # 3프레임 연속 감지
                        rospy.loginfo("로터리 접근 감지! WAIT_ROUNDABOUT 상태로 전이")
                        #self.speed_msg.data = 0.0
                        self.state = State.WAIT_ROUNDABOUT
                        self.state_start = now
                        self.roundabout_detect_count = 0  # 초기화
                else:
                    self.roundabout_detect_count = 0  # 조건 미달시 초기화
            else:
                self.roundabout_detect_count = 0  # 클러스터 없으면 초기화
                rospy.logwarn("[로터리 체크] 현재 클러스터가 하나도 감지되지 않음! (len(self.current_clusters)=0)")
                self.roundabout_detect_count = 0  # 초기화(필요 시)

            if min_front < 2.0:
                # 정면 장애물 감지 시, STOP 상태로 진입해서 분석 시작!
                self.state = State.STOP
                self.state_start = now
                self.stop_analysis_start = now
                self.stop_ranges_log = []
                rospy.loginfo("⏸️ 장애물 감지 → STOP 상태 진입(동/정 분석)")

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
            gap_ok, _ = self.check_gap_around(self.current_clusters, min_gap=1.5, center_angle=160, width=40)
            if gap_ok:
                self.state = State.CHECK_GAP
                self.state_start = now

        elif self.state == State.CHECK_GAP:
            gap_ok, target_vehicle = self.check_gap_around(self.current_clusters, min_gap=2.0, center_angle=160, width=40)
            if gap_ok and target_vehicle is not None:
                self.state = State.ENTER_ROUNDABOUT
                self.target_follow_xy = target_vehicle
                self.state_start = now

        # ========== STOP: 정지 후 장애물 분석 ==========
        # STOP 상태에서 관찰할 때, 모든 프레임 변화량을 기록!
        elif self.state == State.ENTER_ROUNDABOUT:
            now = rospy.get_time()

            # === 1) 이상적 원 궤적 목표 ===
            R = 1.8
            if not hasattr(self, 'enter_roundabout_angle'):
                self.enter_roundabout_angle = 0.0
            self.enter_roundabout_angle += np.deg2rad(18)  # 프레임당 18도 (튜닝)
            if self.enter_roundabout_angle > 2*np.pi:
                self.enter_roundabout_angle -= 2*np.pi

            waypoint_x = R * np.cos(self.enter_roundabout_angle)
            waypoint_y = R * np.sin(self.enter_roundabout_angle)

            # === 2) 리더(동적 클러스터) 찾기 ===
            lead_xy = self.get_dynamic_lead_vehicle_xy(self.prev_clusters, move_thresh=0.08)
            if lead_xy is None and hasattr(self, "target_follow_xy"):
                lead_xy = self.target_follow_xy
            
            print(f"[DEBUG] lead_xy: {lead_xy}")  # ← 여기로 이동

            # === 3) 목표점: 원 궤적 vs 리더 가중합 ===
            weight_my, weight_lead = 0.7, 0.3
            if lead_xy is not None:
                target_x = weight_my * waypoint_x + weight_lead * lead_xy[0]
                target_y = weight_my * waypoint_y + weight_lead * lead_xy[1]
            else:
                target_x, target_y = waypoint_x, waypoint_y

            # === 4) 조향 적용 ===
            self.steer_to((target_x, target_y))

            # === 5) 속도: ACC(간격 + 상대속도 + TTC) ===
            dt = self.lidar_period if self.lidar_period > 0 else 0.1

            if lead_xy is not None:
                lead_dist = float(np.hypot(lead_xy[0], lead_xy[1]))

                # 상대속도(닫힘 속도) 추정 (+면 접근 중)
                if self.prev_lead_dist is not None:
                    rel_speed = (self.prev_lead_dist - lead_dist) / dt
                else:
                    rel_speed = 0.0
                self.prev_lead_dist = lead_dist

                # 앞차 속도 추정 + 필터
                raw_lead_speed = self.get_lead_vehicle_speed()  # [m/s] 가정
                self.lead_speed_lpf = self._lpf(self.lead_speed_lpf, raw_lead_speed, self.lpf_alpha)

                # 내 차 속도(없으면 conservative)
                v_ego = max(0.0, self.current_speed)

                # 목표 간격
                desired_gap = self.s0 + self.T_headway * v_ego

                # TTC 세이프가드
                if rel_speed > 0.05:  # 5cm/s 이상 접근
                    ttc = lead_dist / rel_speed
                else:
                    ttc = 999.0

                # 기본 명령 생성: 간격/상대속도 기반 보정
                gap_error = lead_dist - desired_gap  # +: 여유, -: 과근접
                v_cmd = self.v_cmd_cruise \
                        + self.k_gap * gap_error \
                        - self.k_rel * max(0.0, rel_speed)   # 접근 중이면 감속 가중

                # TTC 급제동
                if ttc < self.ttc_thresh:
                    v_cmd = min(v_cmd, self.v_cmd_min)

                # 한계/저크 제한
                v_cmd = self._clamp(v_cmd, self.v_cmd_min, self.v_cmd_max)
                v_cmd = self._clamp(v_cmd,
                                    self.v_cmd_prev - self.max_delta_cmd,
                                    self.v_cmd_prev + self.max_delta_cmd)
                self.speed_msg.data = v_cmd
                self.v_cmd_prev = v_cmd

                lead_dist = np.sqrt(lead_xy[0]**2 + lead_xy[1]**2)
                angle_deg = (np.rad2deg(np.arctan2(lead_xy[1], lead_xy[0])) + 360) % 360
                
                # 거리 + 각도 조건
                if lead_dist < (self.s0 + 0.2) and 160 <= angle_deg <= 190:
                    rospy.logwarn(f"🚨 매우 근접({lead_dist:.2f} m, angle={angle_deg:.1f}°) → 강한 감속")
                    self.speed_msg.data = self.v_cmd_min

            else:
                # 리더가 없으면 제한된 순항
                v_cmd = self._clamp(self.v_cmd_cruise, self.v_cmd_min, self.v_cmd_max)
                v_cmd = self._clamp(v_cmd,
                                    self.v_cmd_prev - self.max_delta_cmd,
                                    self.v_cmd_prev + self.max_delta_cmd)
                self.speed_msg.data = v_cmd
                self.v_cmd_prev = v_cmd

            # === 6) ‘거리 보존’ 기반 탈출 타이밍(실시간 스케일링) ===
            # 현재 명령 속도 기준으로 t_need 재계산 → 느리면 오래 유지, 빠르면 빨리 탈출
            v_now_cmd = getattr(self.speed_msg, "data", 0.0)  # cmd 단위
            t_need = self.compute_follow_time(v_now_cmd)

            if (now - self.state_start) > t_need:
                self.state = State.EXIT_ROUNDABOUT
                self.state_start = now



        elif self.state == State.EXIT_ROUNDABOUT:
            # roundabout 탈출 판단, 직진 복귀 등
            self.servo_msg.data = 1.0  # 조향(우회전)
            self.speed_msg.data = 1000 # 속도
            now = rospy.get_time()

            # 일정 시간(예: 0.5초) 우회전 유지 후 STRAIGHT로 전이
            if (now - self.state_start) > 0.5:
                self.state = State.STRAIGHT
                self.state_start = now  # STRAIGHT 진입시각 갱신
            
                rospy.loginfo("✅ EXIT_ROUNDABOUT 종료, STRAIGHT 상태로 복귀")
                return

        #elif self.state == State.END:
        #    self.servo_msg.data = 0.5
        #    self.speed_msg.data = 2000



        self.speed_pub.publish(self.speed_msg)
        self.servo_pub.publish(self.servo_msg)
        self.prev_clusters = np.copy(self.current_clusters)

if __name__ == "__main__":
    try:
        node = Control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
