#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32MultiArray, Float64MultiArray, Int16MultiArray
from morai_msgs.msg import GetTrafficLightStatus, ObjectStatusList
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import numpy as np
from enum import Enum
import actionlib
import os

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

class controller :
    def __init__(self):
        rospy.init_node("control_v")
        rospy.Subscriber("/driving_center", Int32MultiArray, self.camCB)
        rospy.Subscriber("/lidar_clusters", Float64MultiArray, self.clusters_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficCB)
        rospy.Subscriber('/delivery_object', ObjectStatusList, self.deliv_objectCB)
        rospy.Subscriber('/delivery_check', Int16MultiArray, self.deliv_checkCB)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.fixed_center = 320
        
        self.only_once_time = False
        self.left_turn_state = False
        self.red_state = False
        self.len_left = 0
        self.len_right = 0
        self.center_index = 0
        self.line_width = 0
        self.line_number_state = False
        # 우회전 시 조향각을 증폭시킬 계수 (1.0이 기본, 클수록 더 많이 꺾음)
        self.turn_gain = 2.0
        # 정지선 인식
        self.stop_flag = 0
        self.stopline_detected = False
        # 차량 정지선 0.3m이내 정지
        # # 좌회전
        # 차량 감속
        rospy.Timer(rospy.Duration(1.0/10), self.timerCB)
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
        self.cluster_center = np.empty((0, 2))
        self.prev_clusters = None
        self.cluster_count = 0
        # Gap & 회전판단 관련
        self.move_threshold = 0.15
        self.roundabout_min = 2
        self.roundabout_max = 3
        
        self.roundabout_detect_count = 0
        self.roundabout_detect_needed = 3  # 3프레임 연속 감지 시 전이(오탐 방지)

        self.entry_xy = None   # 로터리 진입시 차량 위치

        self.in_roundabout_ctx = True  # 라운드어바웃 컨텍스트(인식 이후~탈출 완료 전)
        
        # [ADD] 하드 진입(고정 조향) 관리 변수
        self.use_hard_entry = False
        self.hard_entry_progress = 0.0   # 누적 수행 시간(초)
        self.hard_entry_start = 0.0      # 트리거 시각 (참고용)
        self.hard_resume_pending = False # STOP으로 빠진 뒤 재개 대기 플래그

        # 임계치/명령
        self.hard_pause_dist  = 0.5      # [m] 이보다 가까우면 즉시 중단→STOP
        self.hard_resume_dist = 0.7      # [m] 이 이상 확보되면 재개
        

        self._hard_stop_cooldown_until = 0.0

        # [ADD] 진행 시간 적산용 타임스탬프
        self._hard_last_ts = rospy.get_time()
        
        self.cluster = 0
        self.obj_count = 0

        self.lidar_mode = False
        self.straight_mood = True

        self.roundabout_mood = False
        self._allow_narrow_after = 0.0  # go_straight 이후 'line_width < 200' 허용 시각

        self.tl_index = None
        self.tl_status = None

        ################################################################### slam 판단 상태변수 ##################################################################
        self.slam_state = True
        self.first_ob_state = False
        self.second_ob_state = False
        self.third_ob_state = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.check = [0, 0, 0]
        
        self.os_state = False

        self.stopline_block = False #0909
        ###################### 1, 2차 차선 구분 상태 변수
################################################################## 라이다 주행 변수 선언 ###################################################################
    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))
    def location_xy(self, r, theta_deg):
        theta = np.deg2rad(theta_deg)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y
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
        if not hasattr(self, "cluster_center") or len(self.cluster_center) == 0:
            return None
        min_r = float("inf")
        lead_xy = None
        for x, y in self.cluster_center:
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
            rospy.loginfo(f"{angle_min_deg}°~{angle_max_deg}° 구간 장애물이 모두 {dist_clear_thresh}m 이상 → 틈 판정")
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
###############################################################################################################################################################
    def deliv_objectCB(self, data):
        self.girl_pose = data.pedestrian_list[0]
        self.obstacle_1 = data.obstacle_list[0]
        self.obstacle_2 = data.obstacle_list[1]
    
    def deliv_checkCB(self, data): 
        self.check = data.data
    
    # 신호등 데이터 초기화
    def trafficCB(self, data):
        self.traffic_state = data.trafficLightStatus

    # 라이다 데이터 초기화
    def clusters_callback(self, data):
        ranges_cluster = data.data
        self.remapped_ranges = ranges_cluster[0 : 360]
        
        if not data.data[360 : ]:
            self.cluster_center = np.empty((0, 2))
        else:
            self.cluster_center = ranges_cluster[360 : ]
            self.cluster = len(ranges_cluster[360 : ])

        
        if not data.data[360 : ]:
            self.cluster_center = np.empty((0, 2))
        else:
            clusters = np.array(data.data[360 : ]).reshape(-1, 2)
            self.cluster_center = clusters
            if self.cluster_center.any():
                for x, y in self.cluster_center:
                    r = np.sqrt(x**2 + y**2)
                    angle = np.rad2deg(np.arctan2(y, x)) % 360
                    if 130 <= angle <= 250 and 0.5 < r < 4.0: #4m 이하
                        self.obj_count += 1
    # 카메라 데이터 초기화
    def camCB(self, data):
        if data.data[3] is not None:
            self.line_width = data.data[3]
        else:
            self.line_width = None
        self.center_index = data.data[0]
        self.len_left = data.data[1]
        self.len_right = data.data[2]

    # 물체의 개수로 라이다 or 차선주행 판단
    def timerCB(self, data):
        # ######################################## slam 판단 ##############################################
        if self.client.get_state() == 3 and not self.os_state:
            os.system("rosnode kill /cmd_vel_to_ackermann  /zero_ackermann_cmd  /throttle_interpolator")
            self.os_state = True
            self.slam_state = True
        if not self.slam_state:
            self.navigation()
        # ######################################################### 슬램 목적지 #######################################################
        elif self.slam_state:
            self.stop_line(self.len_left)
            print(f"{self.avoid_count}")
            if self.avoid_count % 2 == 1:
                rospy.loginfo("1차선 주행")
            else:
                rospy.loginfo("2차선 주행")

            if self.lidar_mode:
                print("라이다모드(유지)")
                
                self.control_B()
                return

            # remapped_ranges가 아직 안 들어온 초기 프레임 가드
            if not hasattr(self, "remapped_ranges"):
                print("[TRIG] lidar 아직 미수신")
                return

            # ROI (전방 ±3도)
            lane_center_deg = 180
            roi_width = 3
            roi_start = lane_center_deg - roi_width
            roi_end   = lane_center_deg + roi_width + 1
            front_ranges = self.remapped_ranges[roi_start:roi_end]
            valid_front  = [r for r in front_ranges if 0.5 < r < 5.0]
            min_front    = (np.median(valid_front) if valid_front else 5.0)

            # 이번 프레임에 한해서 섹터 내 장애물 개수 산출(지역변수)
            obs_cnt = 0
            if hasattr(self, "cluster_center") and len(self.cluster_center) > 0:
                for x, y in self.cluster_center:
                    r = (x**2 + y**2) ** 0.5
                    ang = (np.degrees(np.arctan2(y, x)) + 360) % 360
                    if 110 <= ang <= 250 and 0.5 < r < 4.0:
                        obs_cnt += 1

            print(f"[TRIG] obj_count={obs_cnt}, front={min_front:.2f}")

            # ── 라이다 전환 트리거 ──
            if (obs_cnt >= 1) and (min_front < 1.5):
                self.straight_mood = False
                self.lidar_mode = True
                print("라이다모드")
                self.control_B()
                return

            # ── 라운드어바웃 접근 트리거(프레임 누적 카운트) ──
            if (len(self.cluster_center) > 0) and (self.in_roundabout_ctx) and (self.roundabout_mood):
                # obs_cnt는 이미 계산되어 있음
                if 2 <= obs_cnt:
                    self.roundabout_detect_count += 1
                    if self.roundabout_detect_count >= 2:
                        rospy.loginfo("로터리 접근 감지!")
                        self.straight_mood = False
                        self.lidar_mode = True
                        print("라이다모드")
                        self.control_B()
                        self.roundabout_detect_count = 0
                        return
                else:
                    self.roundabout_detect_count = 0

            # ── 그 외: 카메라 모드 ──
            self.straight_mood = True
            print("카메라모드")
            self.cam_drive()
    def navigation(self):
            self.goal_list = list()
            self.start = MoveBaseGoal()
            self.start.target_pose.header.frame_id = 'map'
            self.start.target_pose.pose.orientation.w = 1.0
            # 목적지 객체 생성 후 아래와 같이 좌표 및 회전 방향 즉 바라볼 방향까지 지정해서 여행 계획표에 .append로 추가
            self.goal_list.append(self.start)
            
            # 두번째 목적지에 대해 x,y좌표와 회전 방향및 바라볼 방향까지 저장
            
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.pose.position.x = 17.558996794004475
            self.goal.target_pose.pose.position.y = -9.718547872550658
            self.goal.target_pose.pose.orientation.w = 1.0
            self.goal.target_pose.pose.orientation.z = 0.0
            self.goal_list.append(self.goal)
            self.run()
    def run(self):
        ############################# 테스트 ########################
        if self.client.get_state() != GoalStatus.ACTIVE:
            self.client.send_goal(self.goal_list[1])
    def stop(self):
        self.client.cancel_all_goals()
    # 카메라 기반 제어 코드
    def cam_drive(self):
        print("stopflag = ", self.stop_flag)
        print("center_index = ", self.center_index )
        print("차선 폭 간격", self.line_width)
        if self.stop_flag == 5:
            self.line_number_state = True
            
        if not self.line_number_state: 
            if self.avoid_count % 2 == 1: # 1차선주행
                if self.stop_flag == 0:
                    if self.len_left > 100:
                        self.go_straight()
                    else:
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)

                elif self.stop_flag == 1:
                    if self.len_left > 150 and self.len_right > 150:
                        self.go_straight()
                    else:
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)

                elif self.stop_flag == 2:
                    if self.len_left > 150 and self.len_right > 150:
                        self.go_straight()
                    else:
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)

                elif self.stop_flag == 3:
                    left_steer_angle = self.cal_steer()
                    self.publish(left_steer_angle, 500)
            
                elif self.stop_flag == 4:
                    if not hasattr(self, "entry_once"): self.entry_once = False
                    now = rospy.get_time()

                    # (1) 직진 구간이면 go_straight 실행 + 5초 게이트 설정
                    if self.len_left > 150 and self.len_right > 150:
                        self.go_straight()
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)
                        self._allow_narrow_after = rospy.get_time() + 6.0  # 5초 후부터 <200 허용
                        return
                    
                    # (3) 좁은 간격: go_straight 이후 5초가 지났을 때만 entry_rotate_1 허용
                    if (not self.entry_once) and (self.line_width is not None) and (self.len_left == 0) and (now >= self._allow_narrow_after) and self.len_right > 70:
                        self.entry_rotate_1()
                        self.entry_once = True
                        return

                    # (4) 그 외에는 계속 차선주행(PD)
                    steer_angle = self.cal_steer()
                    self.publish(steer_angle, 500)
    # ################# 2차선 주행 #$###########################
            else:
                if self.stop_flag == 5:
                    if self.len_left > 100:
                        self.go_straight()
                    else:
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)

                elif self.stop_flag == 6:
                    if self.len_left > 150 and self.len_right > 150:
                        self.go_straight()
                    else:
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)

                elif self.stop_flag == 7:
                    if self.len_left > 150 and self.len_right > 150:
                        self.go_straight()
                    else:
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)

                elif self.stop_flag == 8:
                    left_steer_angle = self.cal_steer()
                    self.publish(left_steer_angle, 500)
            
                elif self.stop_flag == 0:
                    if not hasattr(self, "entry_once"): 
                        self.entry_once = False
                    now = rospy.get_time()

                    # (1) 직진 구간이면 go_straight 실행 + 5초 게이트 설정
                    if self.len_left > 150 and self.len_right > 150:
                        self.go_straight()
                        
                        steer_angle = self.cal_steer()
                        self.publish(steer_angle, 500)
                        self._allow_narrow_after = rospy.get_time() + 6.0  # 5초 후부터 <200 허용
                        return

                    # (2) 넓은 간격이면 즉시 entry_rotate_2
                    if (not self.entry_once) and (self.line_width is not None) and (self.line_width > 360):
                        self.entry_rotate_2()
                        self.entry_once = True
                        return

                    # (3) 좁은 간격: go_straight 이후 5초가 지났을 때만 entry_rotate_1 허용
                    if (not self.entry_once) and (self.line_width is not None) and (self.line_width < 50) and (now >= self._allow_narrow_after):
                        self.entry_rotate_1()
                        self.entry_once = True
                        return

                    # (4) 그 외에는 계속 차선주행(PD)
                    steer_angle = self.cal_steer()
                    self.publish(steer_angle, 500)

        elif self.line_number_state:
            if self.stop_flag == 1:                                      
                steer_angle = self.cal_steer()
                self.publish(steer_angle, 500)
            elif self.traffic_state == 4:
                steer_angle = self.cal_steer()
                self.publish(steer_angle, 500)
            elif self.stop_flag == 2:
                if self.traffic_state == 5 or self.traffic_state == 1 or self.traffic_state == 4:
                    self.red_stop()
                    
                elif self.traffic_state == 16 and not self.only_once_time:
                    self.traffic_left_turn()
                    
                elif self.traffic_state == 33 and not self.only_once_time: 
                    self.traffic_left_turn_2()   
                    
                else:
                    steer_angle = self.cal_steer()
                    self.publish(steer_angle, 500)

            elif self.stop_flag == 3:
                steer_angle = self.cal_steer()
                self.publish(steer_angle, 500)
                
            elif self.stop_flag == 4:
                steer_angle = self.cal_steer()
                self.publish(steer_angle, 500)
                
            elif self.stop_flag == 9:
                if self.len_left > 150 and self.len_right > 150:
                    self.go_straight()
                else:
                    steer_angle = self.cal_steer()
                    self.publish(steer_angle, 500)
                    
    def red_stop(self):
        rate = rospy.Rate(10) 
        start_time = rospy.get_time()
        duration = 0.2
        
        if not self.red_state:
            while rospy.get_time() - start_time < duration:
                self.publish(0.5, 300)
                rate.sleep()
                self.red_state = True
        elif self.red_state:
            self.publish(0.5, 0)
        
    def traffic_left_turn_2(self):
        start_time = rospy.get_time()
        duration = 0.4
        duration_2 = 1.4
        if not self.left_turn_state:
            while rospy.get_time() - start_time < duration:
                self.publish(0.5, 2500)
                self.left_turn_state = True
        elif self.left_turn_state:
            while rospy.get_time() - start_time < duration_2:
                self.publish(0.27, 2500)
                self.only_once_time = True
    
    def traffic_left_turn(self):
        start_time = rospy.get_time()
        duration = 0.5
        duration_2 = 1.4
        if not self.left_turn_state:
            while rospy.get_time() - start_time < duration:
                self.publish(0.5, 2500)
                self.left_turn_state = True
        elif self.left_turn_state:
            while rospy.get_time() - start_time < duration_2:
                self.publish(0.27, 2500)
                self.only_once_time = True
    # 정지선 직진 하드 코딩
    def go_straight(self):
        start_time = rospy.get_time()
        duration = 2.5
        print("직진 하드 코딩")
        
        while (rospy.get_time() - start_time < duration) and self.straight_mood:
            # 1) LiDAR가 켜지면 즉시 중단
            if self.lidar_mode:
                break

            # 2) LiDAR가 안 켜져도, 정면 근접 시 즉시 중단 (비상 브레이크)
            #    ROI: 180±3도, 유효범위 0.5~5.0m
            if hasattr(self, "remapped_ranges"):
                roi = self.remapped_ranges[177:184]
                valid = [r for r in roi if 0.5 < r < 5.0]
                dmin = min(valid) if valid else 5.0
                if dmin < 1.5:  # 임계값은 0.7~1.0m 사이에서 현장 튜닝
                    rospy.logwarn(f"[HC] emergency stop: front={dmin:.2f} m")
                    self.publish(0.5, 0.0)   # 즉시 정지
                    break
            # 정상 직진 유지
            self.publish(0.5, 1500)
            rospy.sleep(0.01)
    # 로터리 구간 진입 좌회전 하드코딩
    def entry_rotate_2(self):
        start_time = rospy.get_time()
        local_round_cnt = 0
        mood_set = False
    
        print("좌회전 하드 코딩")
        while rospy.get_time() - start_time < 5.0:
            t = rospy.get_time() - start_time
            post_gate = (t >= 5.0)  # 진입 초기 오인식 방지 게이트
    
            # ===== 하드코딩 조향/속도 =====
            if t < 1.65:
                self.publish(0.29, 2500)
            elif t < 2.15:
                self.publish(0.5, 2500)
                # (선택) 정말 위험할 때만 비상정지 허용하고, 모드전환은 막아둠
                if hasattr(self, "remapped_ranges"):
                    roi = self.remapped_ranges[177:184]
                    valid = [r for r in roi if 0.5 < r < 5.0]
                    dmin = (np.median(valid) if valid else 5.0)
                    if dmin < 0.8:  # 비상 임계값(현장 튜닝)
                        self.publish(0.5, 0.0)  # 즉시 정지만
            else:
                # 2.2초 지났으니 이제부터 인식/전환 허용
                if not mood_set:
                    self.roundabout_mood = True
                    mood_set = True
                    rospy.loginfo("[ROUND] mood=ON (post 2.2s)")
    
                # ===== 로터리 접근 자체 감지(게이팅 이후에만 수행) =====
                # 정면 ROI
                roi = self.remapped_ranges[177:184] if hasattr(self, "remapped_ranges") else []
                valid = [r for r in roi if 0.5 < r < 5.0]
                min_front = (np.median(valid) if valid else 5.0)
    
                # 섹터 내 클러스터 개수(130~250°, 0.5<r<4.0)
                obs_cnt = 0
                if hasattr(self, "cluster_center") and len(self.cluster_center) > 0:
                    for x, y in self.cluster_center:
                        r = (x**2 + y**2) ** 0.5
                        ang = (np.degrees(np.arctan2(y, x)) + 360) % 360
                        if 130 <= ang <= 250 and 0.5 < r < 4.0:
                            obs_cnt += 1
    
                # 근접 트리거(이제 허용): LiDAR 전환
                if (obs_cnt >= 1) and (min_front < 1.5):
                    self.straight_mood = False
                    self.lidar_mode = True
                    rospy.loginfo("[entry_rotate_2] post-gate 근접 트리거 → LiDAR 전환")
                    self.control_B()
                    return
    
                # 라운드어바웃 접근 누적(게이팅 + 2프레임)
                gated = self.in_roundabout_ctx and self.roundabout_mood and (len(self.cluster_center) > 0)
                if gated and (2 <= obs_cnt <= 5):
                    local_round_cnt += 1
                else:
                    local_round_cnt = 0
    
                if gated and (local_round_cnt >= 2):
                    self.straight_mood = False
                    self.lidar_mode = True
                    rospy.loginfo("[entry_rotate_2] post-gate 누적 감지 → LiDAR 전환")
                    self.control_B()
                    return
                # 게이팅 이후엔 대기 시 정지 유지
                self.publish(0.5, 0)
            rospy.sleep(0.01)
    
    def entry_rotate_1(self):
        start_time = rospy.get_time()
        local_round_cnt = 0
        mood_set = False

        print("좌회전 하드 코딩")
        while rospy.get_time() - start_time < 5.0:
            t = rospy.get_time() - start_time
            post_gate = (t >= 4.0)  # 1.4s 이후부터만 인식/전환 허용

            # ===== 하드코딩 조향/속도 =====
            if t < 1.2:
                self.publish(0.2, 2500)
            elif t < 1.8:
                self.publish(0.5, 2500)
            else:
                self.publish(0.5, 0)  # 대기 정지

            # ===== 게이트 이전: 전환 금지(비상 정지만 허용) =====
            if not post_gate:
                if hasattr(self, "remapped_ranges"):
                    roi = self.remapped_ranges[177:184]
                    valid = [r for r in roi if 0.5 < r < 5.0]
                    dmin = (np.median(valid) if valid else 5.0)
                    if dmin < 0.8:  
                        self.publish(0.5, 0.0)  # 즉시 정지
                rospy.sleep(0.01)
                continue

            # ===== 게이트 이후: mood ON + 인식/전환 허용 =====
            if not mood_set:
                self.roundabout_mood = True
                mood_set = True
                rospy.loginfo("[ROUND] mood=ON (post 1.4s)")

            # 정면 ROI
            roi = self.remapped_ranges[177:184] if hasattr(self, "remapped_ranges") else []
            valid = [r for r in roi if 0.5 < r < 5.0]
            min_front = (np.median(valid) if valid else 5.0)

            # 섹터 내 클러스터 개수(130~250°, 0.5<r<4.0)
            obs_cnt = 0
            if hasattr(self, "cluster_center") and len(self.cluster_center) > 0:
                for x, y in self.cluster_center:
                    r = (x**2 + y**2) ** 0.5
                    ang = (np.degrees(np.arctan2(y, x)) + 360) % 360
                    if 130 <= ang <= 250 and 0.5 < r < 4.0:
                        obs_cnt += 1

            # (A) 근접 트리거: LiDAR 전환
            if (obs_cnt >= 1) and (min_front < 1.5):
                self.straight_mood = False
                self.lidar_mode = True
                rospy.loginfo("[entry_rotate_1] post-gate 근접 트리거 → LiDAR 전환")
                self.control_B()
                return

            # (B) 라운드어바웃 접근 누적(게이팅 + 2프레임)
            gated = self.in_roundabout_ctx and self.roundabout_mood and (len(self.cluster_center) > 0)
            if gated and (2 <= obs_cnt <= 5):
                local_round_cnt += 1
            else:
                local_round_cnt = 0

            if gated and (local_round_cnt >= 2):
                self.straight_mood = False
                self.lidar_mode = True
                rospy.loginfo("[entry_rotate_1] post-gate 누적 감지 → LiDAR 전환")
                self.control_B()
                return

            rospy.sleep(0.01)

    def Traffic_light_2(self):
        start_time = rospy.get_time()
        local_round_cnt = 0
        
        while rospy.get_time() - start_time < 1.15:
            t = rospy.get_time() - start_time
            
            # ===== 하드코딩 조향/속도 =====
            if t < 0.3:
                self.publish(0.5, 2500)
            else:
                self.publish(0.9, 2500)

    def Traffic_light_1(self):
        start_time = rospy.get_time()
        duration = 2.0
        print("직진 하드 코딩")

        while (rospy.get_time() - start_time < duration) and self.straight_mood:
        # 정상 직진 유지
            self.publish(0.33, 2500)
            rospy.sleep(0.01)

    # 정지선 인식 flag증가 함수
    def stop_line(self, l):
        if getattr(self, "stopline_block", False):
            return
        if self.len_left > 150 and self.len_right > 150:
            stop_line_status = True
        else:
            stop_line_status = False
            
        if stop_line_status and not self.stopline_detected:
            self.stopline_detected = True
            self.stop_flag += 1
        elif not stop_line_status:
            self.stopline_detected = False
        else:
            pass
    
    def cal_steer(self):
        self.last_err_px = getattr(self, "last_err_px", 0.0)
        self.steer_hist = getattr(self, "steer_hist", [0.0]*5)
        self.prev_steer = getattr(self, "prev_steer", 0.5)

        err = float(self.center_index - self.fixed_center)  # +: 우측, -: 좌측 오차

        # 우회전 곡률 프록시(프레임간 변화 ema
        self.dk_ema = getattr(self, "dk_ema", 0.0)
        beta = 0.35  #0.25 -0.45  EMA 계수
        dk = err - self.last_err_px         # 이번 프레임과 직전 프레임의 오차차
        self.dk_ema = (1.0 - beta) * self.dk_ema + beta * dk
        kappa_r = max(0.0, self.dk_ema)   # 우회전 성분만

        # === 우회전 강회전(ultra) 모드 히스테리시스 ===
        #  - on: err가 크거나(>90px) 곡률 프록시가 큼(>6)
        #  - off: 곡률이 충분히 줄어듦(<3) + err도 완화(<40)
        self.right_ultra = getattr(self, "right_ultra", False)
        if (err > 90.0 and err > 0) or (kappa_r > 6.0):
            self.right_ultra = True                      # ultra-right-turn: on
        elif (kappa_r < 3.0 and err < 40.0):
            self.right_ultra = False                     # ultra-right-turn: off

        # P
        norm = abs(err) / 160.0
        sig = 1.0 / (1.0 + np.exp(-norm))
        divisor = 200.0 + sig * (360.0 - 200.0)

        # 우회전 값이 클때 P를 더 키움(divisor)
        if self.right_ultra and err > 0.0:
            divisor *= 0.6   # (0.55~0.70 사이 작을수록 더 강하게 꺾음)

        #D 우/좌 비대칭
        Kd_right = 0.0030
        Kd_left  = 0.0015
        Kd = Kd_right if err > 0.0 else Kd_left

        p_term = err / divisor
        d_term = Kd * (err - self.last_err_px)

        # ultra-right-turn: 우회전 클떄 P를 더 키움(divisor 갑소)
        FF_GAIN = 0.22       # (0.16~0.28) 크게 할수록 더 빨리 오른쪽으로
        FF_ALPHA = 1.0/22.0  # (1/18~1/30) 작을수록 민감
        ff_term = 0.0      #기본 0
        if err > 0.0:
            ff_term = FF_GAIN * np.tanh(FF_ALPHA * kappa_r)

        steer_delta = p_term + d_term + ff_term

        # ---- 이동평균(노이즈 억제) ----
        self.steer_hist.append(steer_delta)
        if len(self.steer_hist) > 5:
            self.steer_hist.pop(0)
        filt_delta = float(np.mean(self.steer_hist))

        target = 0.5 + filt_delta

        # === 포화 한계 ===
        right_lim = 1.0      # ultra-right-turn: 우측 상한 1.0 허용
        left_lim  = 0.1
        steer_lin = max(left_lim, min(right_lim, target))

        # ultra-right-turn: 우측 증가 슬루(프레임당 증가 허용량) 완화
        #   강회전일 때는 빠르게 1.0에 접근, 평소에는 보수적으로
        SLEW_RIGHT = 0.16 if (self.right_ultra and err > 0.0) else 0.04
        steer = steer_lin
        if steer > self.prev_steer:   # 우측으로 커질 때만 제한(좌측은 기존 유지)
            steer = min(steer, self.prev_steer + SLEW_RIGHT)

        # (선택) 매우 급한 상황(곡률/오차가 큰 프레임)엔 한 번 더 밀어줌
        if self.right_ultra and err > 0.0 and (kappa_r > 18.0 or err > 140.0):
            steer = min(1.0, max(steer, self.prev_steer + 0.22))  # 한 프레임 가속

        # ---- 상태 업데이트 ----
        self.last_err_px = err
        self.prev_steer  = steer

        # (선택) 속도 스케줄: 급우회전에서 자동 감속/가속 (하드코딩 아님)
        vmax, vmin = 2500.0, 800.0
        C = 0.02  # (0.015~0.03; 크면 코너에서 더 감속)
        self.speed_cmd = float(np.clip(vmax / (1.0 + C * kappa_r), vmin, vmax))

        return steer

    # 라이다 기반 제어 코드
    def control_B(self):
        # [ADD] dt 계산 (하드 진입 progress 적산용)
        now = rospy.get_time()
        dt = max(0.0, now - getattr(self, "_hard_last_ts", now))
        self._hard_last_ts = now
        
        # ROI
        lane_center_deg = 180
        roi_width = 3
        roi_start = lane_center_deg - roi_width
        roi_end = lane_center_deg + roi_width + 1
        front_ranges = self.remapped_ranges[roi_start:roi_end]
        valid_front = [r for r in front_ranges if 0.5 < r < 5.0]
        min_front = min(valid_front) if valid_front else 5.0
        
        if valid_front:
            min_idx = np.argmin(front_ranges)
            min_dist = front_ranges[min_idx]
            min_angle = roi_start + min_idx

        # ========== STRAIGHT ==========
        if self.state == State.STRAIGHT:
            self.speed_msg.data = 2000.0
            self.steer_msg.data = 0.5

            # 클러스터가 있을 때만 각도 계산 및 라운드어바웃 접근 판별
            if (len(self.cluster_center) > 0) and (self.in_roundabout_ctx):
                # 거리와 각도 모두 만족하는 클러스터만 카운트
                self.obj_count = 0
                for x, y in self.cluster_center:
                    r = np.sqrt(x**2 + y**2)
                    angle = np.rad2deg(np.arctan2(y, x)) % 360
                    if 130 <= angle <= 250 and 0.5 < r < 4.0: #4m 이하
                        self.obj_count += 1
                if 2 <= self.obj_count <= 5:
                    self.roundabout_detect_count += 1
                    if self.roundabout_detect_count >= 2:  # 3프레임 연속 감지
                        rospy.loginfo("로터리 접근 감지! WAIT_ROUNDABOUT 상태로 전이")
                        self.state = State.WAIT_ROUNDABOUT
                        self.state_start = now
                        self.roundabout_detect_count = 0

                else:
                    self.roundabout_detect_count = 0
                    
                if (min_front < 2.0) :  # [MOD]
                    self.state = State.STOP
                    self.state_start = now
                    self.stop_analysis_start = now
                    self.stop_ranges_log = []
                    rospy.loginfo("장애물 감지 → STOP 상태 진입(동/정 분석)")
                
            else:
                self.roundabout_detect_count = 0
                rospy.logwarn("[로터리 체크] 현재 클러스터가 하나도 감지되지 않음! (len(self.cluster_center)=0)")
        
        elif self.state == State.STOP:
            self.speed_msg.data = 0.0
            self.steer_msg.data = 0.5

            # 하드 진입 일시정지에서 왔다면: 안전거리 확보 시 즉시 재개
            if getattr(self, "hard_resume_pending", False):
                sector = self._clusters_in_sector(self.cluster_center, 130, 250)
                min_r = min([r for _,_,_,r in sector]) if sector else float('inf')
                if min_r >= self.hard_resume_dist:
                    rospy.loginfo(
                        f"🟢 하드 진입 재개: min_r={min_r:.2f}m ≥ {self.hard_resume_dist:.1f}m"
                    )
                    self.use_hard_entry = True
                    # 진행시간은 유지(이어달리기)
                    self.hard_resume_pending = False
                    self.state = State.ENTER_ROUNDABOUT
                    self.stopline_block = True #0909
                    self.state_start = now
                    return
            # 재개 조건 안 되면 그냥 정지 유지
            else:
                # 기존 동/정 분석(라운드어바웃 컨텍스트가 아닐 때만)
                self.stop_ranges_log.append(self.remapped_ranges[140:220])
                if now - self.stop_analysis_start > self.stop_analysis_duration:
                    if len(self.stop_ranges_log) > 0.5:
                        start_ranges = self.stop_ranges_log[0]
                        end_ranges   = self.stop_ranges_log[-1]
                        diffs = np.abs(np.array(end_ranges) - np.array(start_ranges))
                        moving_score = np.mean(diffs)
                    else:
                        moving_score = 0.0

                    rospy.loginfo(f"[STOP분석] moving_score={moving_score:.4f}")
                    if moving_score > 0.3:
                        
                        rospy.loginfo("동적 장애물로 판단 → DYNAMIC_STOP 상태")
                        self.state = State.DYNAMIC_STOP
                        self.state_start = now
                        self.last_dynamic_seen_time = now
                    else:
                        rospy.loginfo("정적 장애물로 판단 → 회피 진입")
                        self.state = State.AVOIDING
                        self.state_start = now
                        self.avoid_start = now

        elif self.state == State.WAIT_ROUNDABOUT:
            self.speed_msg.data = 0.0
            self.steer_msg.data = 0.5

            sector_min = 130
            sector_max = 250
        
            # 쿨다운 중이면 대기
            if now < getattr(self, "_hard_stop_cooldown_until", 0.0):
                self.speed_pub.publish(self.speed_msg)
                self.steer_pub.publish(self.steer_msg)
                self.prev_clusters = np.copy(self.cluster_center)
                return

            sector = self._clusters_in_sector(self.cluster_center, sector_min, sector_max)
            min_r = min([r for _, _, _, r in sector]) if sector else float('inf')
            # --- 이어하기 재개 우선 ---
            if self.hard_resume_pending and (min_r >= self.hard_resume_dist):
                rospy.loginfo(f"하드 진입 재개: 남은 {max(0.0, self.hard_entry_duration - self.hard_entry_progress):.2f}s")
                self.use_hard_entry = True
                # progress는 유지! 리셋 금지
                self.state = State.ENTER_ROUNDABOUT
                self.state_start = now
                self.hard_resume_pending = False
                return
            # (옵션) 기존 gap 체크
            gap_ok, _ = self.check_gap_around(self.cluster_center)
            opening = self._sector_opening(self.prev_clusters, self.cluster_center, sector_min, sector_max, open_thresh=0.05)
            # --- 새로 시작 트리거(처음부터) ---
            if min_r >= 1.0:
                self.use_hard_entry = True
                self.hard_entry_progress = 0.0     # 새 시작일 때만 리셋
                self.hard_entry_start = now
                self.state = State.ENTER_ROUNDABOUT
                self.stopline_block = True #0909
                self.state_start = now
                rospy.loginfo(f"🚗 하드 진입 트리거: 섹터 min_r={min_r:.2f}m ≥ 2.0m → 하드모드 시작")
            elif gap_ok and opening:
                rospy.loginfo(f"GAP OK (min_r={min_r:.2f}m, opening={opening}) → CHECK_GAP")
                self.state = State.CHECK_GAP
                self.state_start = now
        
        elif self.state == State.CHECK_GAP:
            # 바운싱 방지용 재확인
            self.speed_msg.data = 0.0
            self.steer_msg.data = 0.5

            sector = self._clusters_in_sector(self.cluster_center, 130, 250)
            min_r = min([r for _, _, _, r in sector]) if sector else float('inf')
            opening = self._sector_opening(self.prev_clusters, self.cluster_center, 130, 250, open_thresh=0.05)

            if (min_r >= 2.0) and opening:
                rospy.loginfo(f"🚗 로터리 진입 시작 (min_r={min_r:.2f}m, opening={opening})")
                rospy.loginfo(f"5555555555555555555555555555555555555555")
                self.state = State.ENTER_ROUNDABOUT
                self.stopline_block = True
                self.state_start = now
            else:
                # 조건 깨지면 다시 대기
                self.state = State.WAIT_ROUNDABOUT
                self.state_start = now

        elif self.state == State.ENTER_ROUNDABOUT:
            self.stopline_block = True #0909

            # ── 하드 진입 모드 우선 처리 ──
            if getattr(self, "use_hard_entry", False):
                # 현재 160~190° 섹터 최소거리
                sector = self._clusters_in_sector(self.cluster_center, 130, 250)
                min_r = min([r for _, _, _, r in sector]) if sector else float('inf')
                
                # (A) 위험하면 즉시 중단 → STOP으로 이동 (일시정지)
                if min_r < self.hard_pause_dist:
                    rospy.logwarn(f"하드 진입 중단: min_r={min_r:.2f}m < {self.hard_pause_dist:.1f}m → STOP 대기")
                    self.hard_resume_pending = True
                    self.use_hard_entry = False
                    self.state = State.STOP
                    self.state_start = now
                    # 즉시 정지 명령 내보내기
                    self.speed_msg.data = 0.0
                    self.steer_msg.data = 0.5
                    self.speed_pub.publish(self.speed_msg)
                    self.steer_pub.publish(self.steer_msg)
                    # 너무 빨리 재트리거 되는 걸 방지하는 짧은 쿨다운
                    self._hard_stop_cooldown_until = now + 0.2
                    return

                # (B) 정상 진행: 고정 조향/속도, 진행시간 적산

                self.avoid_count += 0   
                if self.avoid_count % 2 == 0:
                    self.hard_entry_duration =2.2
                    #2차선
                    self.steer_msg.data = 0.76    # 예: 0.7 
                    self.speed_msg.data = 1200.0     # 예: 1200.0
                    self.hard_entry_progress += dt            # 진행시간 누적

                else:
                    self.hard_entry_duration =2.2 
                    self.steer_msg.data = 0.7     # 예: 0.7 
                    self.speed_msg.data = 1200.0     # 예: 1200.0
                    self.hard_entry_progress += dt            # 진행시간 누적

                
                
                # 완료되면 EXIT로
                if self.hard_entry_progress >= self.hard_entry_duration:  # 예: 2.0초
                    rospy.loginfo("하드 진입 완료 → EXIT_ROUNDABOUT 전이")
                    self.use_hard_entry = False
                    self.hard_resume_pending = False
                    self.hard_entry_progress = 0.0
                    self.state = State.EXIT_ROUNDABOUT
                    self.state_start = now

                # 하드 모드일 땐 아래 일반 로직 타지 않음(즉시 반환)
                self.speed_pub.publish(self.speed_msg)
                self.steer_pub.publish(self.steer_msg)
                self.prev_clusters = np.copy(self.cluster_center)
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
            rospy.loginfo("[AVOIDING_RIGHT] 하드코딩 수행")
            t0 = rospy.get_time()
            while rospy.get_time() - t0 < 2.7:  # 총 1.3초
                elapsed = rospy.get_time() - t0
                if elapsed < 1.35:       # 0~0.4s: 우측 회피
                    steer = 0.1
                elif elapsed < 2.7:     # 0.4~0.9s: 직진
                    steer = 0.9
                self.steer_msg.data = steer
                self.speed_msg.data = 500.0
                self.speed_pub.publish(self.speed_msg)
                self.steer_pub.publish(self.steer_msg)
                rospy.sleep(0.01)

            self.state = State.STRAIGHT
            self.lidar_mode = False 
            self.state_start = rospy.get_time()
            rospy.loginfo("하드코딩 우측 회피 완료 → STRAIGHT 복귀")
            return
        elif self.state == State.AVOIDING_RIGHT:
            rospy.loginfo("[AVOIDING_RIGHT] 하드코딩 수행")
            t0 = rospy.get_time()
            while rospy.get_time() - t0 < 2.7:  # 총 1.3초
                elapsed = rospy.get_time() - t0
                if elapsed < 1.35:       # 0~0.4s: 우측 회피
                    steer = 0.9
                elif elapsed < 2.7:     # 0.4~0.9s: 직진
                    steer = 0.1
                self.steer_msg.data = steer
                self.speed_msg.data = 500.0
                self.speed_pub.publish(self.speed_msg)
                self.steer_pub.publish(self.steer_msg)
                rospy.sleep(0.01)
            self.state = State.STRAIGHT
            self.lidar_mode = False 
            self.state_start = rospy.get_time()
            rospy.loginfo("하드코딩 우측 회피 완료 → STRAIGHT 복귀")
            return
        elif self.state == State.RETURNING:
            self.speed_msg.data = 500.0
            if self.return_index >= 0:
                recovery_val = 0.5 + (0.5 - self.reduced_log[self.return_index])
                self.steer_msg.data = float(np.clip(recovery_val, 0.0, 1.0))
                rospy.loginfo(f"[RETURNING] 복귀 인덱스 {self.return_index} | 조향값: {self.steer_msg.data:.2f}")
                self.return_index -= 1
            else:
                self.steer_msg.data = 0.7
                self.state = State.STRAIGHT
                self.lidar_mode = False 
                self.state_start = now
                rospy.loginfo("2차선 복귀 (STRAIGHT)")
        elif self.state == State.DYNAMIC_STOP:
            self.speed_msg.data = 0.0
            self.steer_msg.data = 0.5
            if now - self.last_dynamic_seen_time > 1.0:
                rospy.loginfo("동적 장애물 사라짐 → STRAIGHT 상태로 복귀")
                self.state = State.STRAIGHT
                self.lidar_mode = False
                self.state_start = now
        elif self.state == State.EXIT_ROUNDABOUT:
            # roundabout 탈출 판단, 직진 복귀 등
            self.steer_msg.data = 0.7  # 조향(우회전)
            self.speed_msg.data = 1400 # 속도
            # 일정 시간(예: 0.5초) 우회전 유지 후 STRAIGHT로 전이
            if (now - self.state_start) > 0.5:
                self.state = State.STRAIGHT
                self.state_start = now  # STRAIGHT 진입시각 갱신
                self.in_roundabout_ctx = False  # [MOD] 컨텍스트 종료
                self.lidar_mode = False 
                self.roundabout_mood = False
                self.EXIT_stopline_count =True
                self.stopline_block = False 

                rospy.loginfo("EXIT_ROUNDABOUT 종료, STRAIGHT 상태로 복귀")
                return

        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)
        self.prev_clusters = np.copy(self.cluster_center)
        self._last_lidar_ts = rospy.get_time()
    def publish(self, steer, speed):
        steering_angle = steer
        speed_data = speed
        self.steer_msg.data = steering_angle
        self.speed_msg.data = speed_data
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)
    
def main():
    try:
        ctrl = controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()