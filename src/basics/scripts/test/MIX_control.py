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

        rospy.Subscriber("/driving_center", Int32, self.camera_lane_cb)

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

        self.in_roundabout_ctx = False  # 라운드어바웃 컨텍스트(인식 이후~탈출 완료 전)

        # [ADD] 하드 진입(고정 조향) 관리 변수
        self.use_hard_entry = False
        self.hard_entry_duration = 1.9   # 총 수행 시간(초)
        self.hard_entry_progress = 0.0   # 누적 수행 시간(초)
        self.hard_entry_start = 0.0      # 트리거 시각 (참고용)
        self.hard_resume_pending = False # STOP으로 빠진 뒤 재개 대기 플래그

        # 임계치/명령
        self.hard_pause_dist  = 0.5      # [m] 이보다 가까우면 즉시 중단→STOP
        self.hard_resume_dist = 0.7      # [m] 이 이상 확보되면 재개
        self.hard_servo = 0.8            # 하드 진입 조향
        self.hard_speed = 1200.0         # 하드 진입 속도

        self._hard_stop_cooldown_until = 0.0

        # [ADD] 진행 시간 적산용 타임스탬프
        self._hard_last_ts = rospy.get_time()


    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

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
    
    #Gap틈 체크 함수
    def check_gap_around(self, clusters, min_gap=1.0, center_angle=190, width=120):
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

    #x,y,각도,거리 반환
    def _clusters_in_sector(self, clusters, angle_min_deg=130, angle_max_deg=250):
        """160~190° 섹터 추출: (x, y, angle_deg, range) 리스트"""
        sector = []
        for x, y in clusters:
            ang = (np.rad2deg(np.arctan2(y, x)) + 360) % 360
            if angle_min_deg <= ang <= angle_max_deg:
                sector.append((x, y, ang, np.hypot(x, y)))
        return sector

    # 틈이 있는지 판단
    def _sector_opening(self, prev_clusters, curr_clusters, angle_min_deg=130, angle_max_deg=250, open_thresh=0.05, dist_clear_thresh=2.0):

        if prev_clusters is None or len(prev_clusters) == 0 or len(curr_clusters) == 0:
            return False

        prev_sector = self._clusters_in_sector(prev_clusters, angle_min_deg, angle_max_deg)
        curr_sector = self._clusters_in_sector(curr_clusters, angle_min_deg, angle_max_deg)

        # (1) 현재 섹터가 완전히 비었을 때 → 틈
        if len(curr_sector) == 0:
            return True

        # (2) 현재 섹터 모든 장애물이 dist_clear_thresh 이상 떨어져 있으면 → 틈
        curr_ranges = [r for _, _, _, r in curr_sector]
        if all(r >= dist_clear_thresh for r in curr_ranges):
            rospy.loginfo(f"✅ {angle_min_deg}°~{angle_max_deg}° 구간 장애물이 모두 {dist_clear_thresh}m 이상 → 틈 판정")
            return True

        # (3) 거리 변화량 기반 판정
        deltas = []
        prev_ranges = [r for _, _, _, r in prev_sector] if prev_sector else []
        for _, _, _, r_now in curr_sector:
            if not prev_ranges:
                deltas.append(0.0)
            else:
                r_prev = min(prev_ranges, key=lambda rp: abs(rp - r_now))
                deltas.append(r_now - r_prev)

        if not deltas:
            return False
        return (np.mean(deltas) > open_thresh)

    def lane_center_cb(self, msg: Int32):
        #카메라에서 받은 차선 중심 좌표를 조향값으로 변환
        
        # 중심오차계산(1 정규화)

        # 조향값계산(0.5 기준, 좌/우 보정)

        pass


    def control_A(self, data):
        rospy.loginfo_once("✅ control_A 실행됨")

        # [ADD] dt 계산 (하드 진입 progress 적산용)
        now = rospy.get_time()
        dt = max(0.0, now - getattr(self, "_hard_last_ts", now))
        self._hard_last_ts = now
        
        # ROI
        remapped_ranges = np.array(data.data)
        lane_center_deg = 180
        roi_width = 3
        roi_start = lane_center_deg - roi_width
        roi_end = lane_center_deg + roi_width + 1
        front_ranges = remapped_ranges[roi_start:roi_end]
        valid_front = [r for r in front_ranges if 0.5 < r < 5.0]
        min_front = min(valid_front) if valid_front else 5.0
        
        if valid_front:
            min_idx = np.argmin(front_ranges)
            min_dist = front_ranges[min_idx]
            min_angle = roi_start + min_idx
            #x, y = self.location_xy(min_dist, min_angle)
        num_close_objects = sum(1 for r in valid_front if r <= 2.0)


        # ========== STRAIGHT ==========
        if self.state == State.STRAIGHT:

            use_lane = False
            use_lane = (num_close_objects <= 1) and (not self.in_roundabout_ctx)

            if use_lane:
                # 카메라조향적용
                # 조향값적용

                # 
                # self.speed_pub.publish(self.speed_msg)
                # self.servo_pub.publish(self.servo_msg)
                pass

            # 클러스터가 있을 때만 각도 계산 및 라운드어바웃 접근 판별
            if len(self.current_clusters) > 0:
                # 거리와 각도 모두 만족하는 클러스터만 카운트
                in_range_count = 0
                for x, y in self.current_clusters:
                    r = np.sqrt(x**2 + y**2)
                    angle = np.rad2deg(np.arctan2(y, x)) % 360
                    if 130 <= angle <= 250 and 0.5 < r < 4.0: #4m 이하
                        in_range_count += 1

                # 2~4개 감지되고, 3프레임 연속이면 WAIT_ROUNDABOUT로 전이
                if 2 <= in_range_count <= 5:
                    self.roundabout_detect_count += 1
                    if self.roundabout_detect_count >= 2:  # 3프레임 연속 감지
                        rospy.loginfo("로터리 접근 감지! WAIT_ROUNDABOUT 상태로 전이")
                        self.state = State.WAIT_ROUNDABOUT
                        self.state_start = now
                        self.roundabout_detect_count = 0
                        self.in_roundabout_ctx = True  # [MOD] 라운드어바웃 컨텍스트 ON

                else:
                    self.roundabout_detect_count = 0
                if (min_front < 2.0) and (not self.in_roundabout_ctx):  # [MOD]
                    self.state = State.STOP
                    self.state_start = now
                    self.stop_analysis_start = now
                    self.stop_ranges_log = []
                    rospy.loginfo("⏸️ 장애물 감지 → STOP 상태 진입(동/정 분석)")
                
            else:
                self.roundabout_detect_count = 0
                rospy.logwarn("[로터리 체크] 현재 클러스터가 하나도 감지되지 않음! (len(self.current_clusters)=0)")
        
        elif self.state == State.STOP:
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5

            if self.in_roundabout_ctx:  # [MOD] 라운드어바웃 모드면 동/정 분석 스킵
                # 하드 진입 일시정지에서 왔다면: 안전거리 확보 시 즉시 재개
                if getattr(self, "hard_resume_pending", False):
                    sector = self._clusters_in_sector(self.current_clusters, 130, 250)
                    min_r = min([r for _,_,_,r in sector]) if sector else float('inf')
                    if min_r >= self.hard_resume_dist:
                        rospy.loginfo(
                            f"🟢 하드 진입 재개: min_r={min_r:.2f}m ≥ {self.hard_resume_dist:.1f}m"
                        )
                        self.use_hard_entry = True
                        # 진행시간은 유지(이어달리기)
                        self.hard_resume_pending = False
                        self.state = State.ENTER_ROUNDABOUT
                        self.state_start = now
                        return
                # 재개 조건 안 되면 그냥 정지 유지
            else:
                # ▼ 기존 동/정 분석(라운드어바웃 컨텍스트가 아닐 때만)
                self.stop_ranges_log.append(remapped_ranges[90:270].copy())
                if now - self.stop_analysis_start > self.stop_analysis_duration:
                    if len(self.stop_ranges_log) > 0.5:
                        start_ranges = self.stop_ranges_log[0]
                        end_ranges   = self.stop_ranges_log[-1]
                        diffs = np.abs(np.array(end_ranges) - np.array(start_ranges))
                        moving_score = np.mean(diffs)
                    else:
                        moving_score = 0.0

                    rospy.loginfo(f"[STOP분석] moving_score={moving_score:.4f}")
                    if moving_score > 0.2:
                        rospy.loginfo("🚨 동적 장애물로 판단 → DYNAMIC_STOP 상태")
                        self.state = State.DYNAMIC_STOP
                        self.state_start = now
                        self.last_dynamic_seen_time = now
                    else:
                        rospy.loginfo("✅ 정적 장애물로 판단 → 회피 진입")
                        self.state = State.AVOIDING
                        self.state_start = now
                        self.avoid_start = now

        elif self.state == State.WAIT_ROUNDABOUT:
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5

            sector_min = 130
            sector_max = 250
        
            # 쿨다운 중이면 대기
            if now < getattr(self, "_hard_stop_cooldown_until", 0.0):
                self.speed_pub.publish(self.speed_msg)
                self.servo_pub.publish(self.servo_msg)
                self.prev_clusters = np.copy(self.current_clusters)
                return
        
            sector = self._clusters_in_sector(self.current_clusters, sector_min, sector_max)
            min_r = min([r for _, _, _, r in sector]) if sector else float('inf')
        
            # --- 이어하기 재개 우선 ---
            if self.hard_resume_pending and (min_r >= self.hard_resume_dist):
                rospy.loginfo(f"▶️ 하드 진입 재개: 남은 {max(0.0, self.hard_entry_duration - self.hard_entry_progress):.2f}s")
                self.use_hard_entry = True
                # ★ progress는 유지! 리셋 금지
                self.state = State.ENTER_ROUNDABOUT
                self.state_start = now
                self.hard_resume_pending = False
                return
        
            # (옵션) 기존 gap 체크
            gap_ok, _ = self.check_gap_around(self.current_clusters)
            opening = self._sector_opening(self.prev_clusters, self.current_clusters, sector_min, sector_max, open_thresh=0.05)
        
            # --- 새로 시작 트리거(처음부터) ---
            if min_r >= 1.0:
                self.use_hard_entry = True
                self.hard_entry_progress = 0.0     # ★ 새 시작일 때만 리셋
                self.hard_entry_start = now
                self.state = State.ENTER_ROUNDABOUT
                self.state_start = now
                rospy.loginfo(f"🚗 하드 진입 트리거: 섹터 min_r={min_r:.2f}m ≥ 2.0m → 하드모드 시작")
            elif gap_ok and opening:
                rospy.loginfo(f"✅ GAP OK (min_r={min_r:.2f}m, opening={opening}) → CHECK_GAP")
                self.state = State.CHECK_GAP
                self.state_start = now
        
        elif self.state == State.CHECK_GAP:
            # 바운싱 방지용 재확인
            self.speed_msg.data = 0.0
            self.servo_msg.data = 0.5

            sector = self._clusters_in_sector(self.current_clusters, 130, 250)
            min_r = min([r for _, _, _, r in sector]) if sector else float('inf')
            opening = self._sector_opening(self.prev_clusters, self.current_clusters, 130, 250, open_thresh=0.05)

            if (min_r >= 2.0) and opening:
                rospy.loginfo(f"🚗 로터리 진입 시작 (min_r={min_r:.2f}m, opening={opening})")
                self.state = State.ENTER_ROUNDABOUT
                self.state_start = now
            else:
                # 조건 깨지면 다시 대기
                self.state = State.WAIT_ROUNDABOUT
                self.state_start = now

        elif self.state == State.ENTER_ROUNDABOUT:
            # ── 하드 진입 모드 우선 처리 ──
            if getattr(self, "use_hard_entry", False):
                # 현재 160~190° 섹터 최소거리
                sector = self._clusters_in_sector(self.current_clusters, 130, 250)
                min_r = min([r for _, _, _, r in sector]) if sector else float('inf')

                # (A) 위험하면 즉시 중단 → STOP으로 이동 (일시정지)
                if min_r < self.hard_pause_dist:
                    rospy.logwarn(f"🚨 하드 진입 중단: min_r={min_r:.2f}m < {self.hard_pause_dist:.1f}m → STOP 대기")
                    self.hard_resume_pending = True
                    self.use_hard_entry = False
                    self.state = State.STOP
                    self.state_start = now
                    # 즉시 정지 명령 내보내기
                    self.speed_msg.data = 0.0
                    self.servo_msg.data = 0.5
                    self.speed_pub.publish(self.speed_msg)
                    self.servo_pub.publish(self.servo_msg)
                    # 너무 빨리 재트리거 되는 걸 방지하는 짧은 쿨다운
                    self._hard_stop_cooldown_until = now + 0.2
                    return

                # (B) 정상 진행: 고정 조향/속도, 진행시간 적산
                self.servo_msg.data = self.hard_servo     # 예: 0.7
                self.speed_msg.data = self.hard_speed     # 예: 1200.0
                self.hard_entry_progress += dt            # ★ 진행시간 누적

                # 완료되면 EXIT로
                if self.hard_entry_progress >= self.hard_entry_duration:  # 예: 2.0초
                    rospy.loginfo("✅ 하드 진입 완료 → EXIT_ROUNDABOUT 전이")
                    self.use_hard_entry = False
                    self.hard_resume_pending = False
                    self.hard_entry_progress = 0.0
                    self.state = State.EXIT_ROUNDABOUT
                    self.state_start = now

                # 하드 모드일 땐 아래 일반 로직 타지 않음(즉시 반환)
                self.speed_pub.publish(self.speed_msg)
                self.servo_pub.publish(self.servo_msg)
                self.prev_clusters = np.copy(self.current_clusters)
                return

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

        elif self.state == State.EXIT_ROUNDABOUT:
            # roundabout 탈출 판단, 직진 복귀 등
            self.servo_msg.data = 0.3  # 조향(우회전)
            self.speed_msg.data = 1000 # 속도

            # 일정 시간(예: 0.5초) 우회전 유지 후 STRAIGHT로 전이
            if (now - self.state_start) > 0.5:
                self.state = State.STRAIGHT
                self.state_start = now  # STRAIGHT 진입시각 갱신
                self.in_roundabout_ctx = False  # [MOD] 컨텍스트 종료
            
                rospy.loginfo("✅ EXIT_ROUNDABOUT 종료, STRAIGHT 상태로 복귀")
                return
    
        self.speed_pub.publish(self.speed_msg)
        self.servo_pub.publish(self.servo_msg)
        self.prev_clusters = np.copy(self.current_clusters)
        self._last_lidar_ts = rospy.get_time()

if __name__ == "__main__":
    try:
        node = Control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
