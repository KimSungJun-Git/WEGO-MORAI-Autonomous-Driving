#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.center_index_pub = rospy.Publisher("/driving_center", Int32MultiArray, queue_size=3)
        # ★ 추가: 차선 대표 점 퍼블리셔
        self.lane_points_pub = rospy.Publisher("/lane_points", Int32MultiArray, queue_size=3)

        self.bridge = CvBridge()
        self.center_index_msg = Int32MultiArray()
        self.lane_points_msg = Int32MultiArray()

        # ====== 기존 값 유지 ======
        self.fixed_center = 320
        self.lane_width = 350

        # ====== 추가: 적응형 HSV 파라미터 ======
        self.base_white = np.array([  0,   0, 192], np.uint8)  # H,S,V_low
        self.base_white_up = np.array([179,  64, 255], np.uint8)
        self.base_yellow = np.array([ 15,  80,   0], np.uint8)
        self.base_yellow_up = np.array([ 40, 255, 255], np.uint8)

        self.ema_alpha = 0.2
        self.v_ema = None
        self.prev_mask = None

        # 정지선(강한 흰색) 모드 관리
        self.stopline_mode = False
        self.stopline_count = 0
        self.stopline_on_threshold = 0.20
        self.stopline_off_threshold = 0.08
        self.hold_frames = 5

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[:2]

        # (1) 조도 적응형 HSV
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        roi_h = int(y * 0.25)
        roi = img_hsv[y - roi_h : y, :]
        v_mean = float(np.mean(roi[:,:,2]))
        if self.v_ema is None:
            self.v_ema = v_mean
        else:
            self.v_ema = (1 - self.ema_alpha) * self.v_ema + self.ema_alpha * v_mean

        v_low = int(np.clip(192 - (200 - self.v_ema) * 0.25, 150, 210))
        white_lower = self.base_white.copy(); white_lower[2] = v_low
        s_low = int(np.clip(80 - (200 - self.v_ema) * 0.15, 60, 80))
        yellow_lower = self.base_yellow.copy(); yellow_lower[1] = s_low

        # (2) 정지선 모드 감지/해제
        band_h = max(20, int(y * 0.10))
        band = img_hsv[y - band_h : y, :]
        band_white = cv2.inRange(band, white_lower, self.base_white_up)
        white_ratio = float(np.count_nonzero(band_white)) / (band.shape[0]*band.shape[1] + 1e-6)

        if not self.stopline_mode and white_ratio >= self.stopline_on_threshold:
            self.stopline_mode = True; self.stopline_count = self.hold_frames
        elif self.stopline_mode:
            self.stopline_count -= 1
            if white_ratio <= self.stopline_off_threshold and self.stopline_count <= 0:
                self.stopline_mode = False

        # (3) HSV 마스크
        white_range = cv2.inRange(img_hsv, white_lower, self.base_white_up)
        yellow_range = cv2.inRange(img_hsv, yellow_lower, self.base_yellow_up)
        if self.stopline_mode:
            combined_range = cv2.bitwise_or(white_range, cv2.bitwise_and(yellow_range, white_range))
        else:
            combined_range = cv2.bitwise_or(yellow_range, white_range)

        # (4) 히스테리시스
        if self.prev_mask is not None:
            combined_range = cv2.bitwise_or(combined_range, self.prev_mask)
        self.prev_mask = combined_range.copy()

        filtered_img = cv2.bitwise_and(img, img, mask=combined_range)

        # (5) Perspective transform (기존 유지)
        src_points = np.float32([[0, 420], [180, 300], [400, 320], [x-40, 420]])
        dst_points = np.float32([[x*0.25, y-1], [x*0.25, 0], [x*0.75, 0], [x*0.75, y-1]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))

        # (6) Binary (기존 유지)
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 50] = 255

        # (7) Histogram 기반 좌/우 검출 (기존 유지)
        histogram = np.sum(bin_img, axis=0)
        left_hist = histogram[0:self.fixed_center]
        right_hist = histogram[self.fixed_center:]
        left_indices = np.where(left_hist > 5)[0]
        right_indices = np.where(right_hist > 5)[0] + self.fixed_center

        center_index = self.judgement(left_indices, right_indices)

        # ★ 추가: 좌/우 대표 점(xl, yl, xr, yr) 산출 (BEV 좌표계, 맨 아래 y)
        yl = y - 1
        yr = y - 1
        xl = int(left_indices[-1]) if len(left_indices) > 0 else -1
        xr = int(right_indices[0]) if len(right_indices) > 0 else -1
        self.publish_points(xl, yl, xr, yr)

        # (8) 기존 퍼블리시 유지
        self.publish(center_index, left_indices, right_indices)

        # 디버깅
        cv2.imshow("mask", combined_range)
        cv2.imshow("Original", img)
        cv2.imshow("Warped", warped_img)
        cv2.waitKey(1)
        
    def judgement(self, l_data, r_data):
        center_index = self.fixed_center
        left_indices = l_data
        right_indices = r_data
        print(f"{self.center_index_msg}")

        if len(left_indices) > 0 and len(right_indices) > 0:
            left_pos = left_indices[-1]
            right_pos = right_indices[0]
            current_width = right_pos - left_pos
            self.lane_width = 0.9 * self.lane_width + 0.1 * current_width
            center_index = (left_pos + right_pos) // 2
            return center_index
        elif len(left_indices) > 0 and len(right_indices) == 0:
            left_pos = left_indices[-1]
            center_index = int(left_pos + self.lane_width / 2)
            return center_index
        elif len(right_indices) > 0 and len(left_indices) == 0:
            right_pos = right_indices[0]
            center_index = int(right_pos - self.lane_width / 2)
            return center_index
        else:
            return center_index

    def publish(self, data, left, right):
        center_index = data
        len_left = len(left)
        len_right = len(right)
        width = None
        print(len_left, len_right)
        if len_left > 0 and len_right > 0:
            width = right[0] - left[-1]
        c_l_r_w = [center_index, len_left, len_right, width]
        self.center_index_msg.data = c_l_r_w
        if width is not None:
            self.center_index_pub.publish(self.center_index_msg)

    # ★ 추가: lane_points 퍼블리시 (xl, yl, xr, yr)
    def publish_points(self, xl, yl, xr, yr):
        # BEV 좌표계 (warped_img 기준)
        self.lane_points_msg.data = [int(xl), int(yl), int(xr), int(yr)]
        # 어떤 경우에도 포인트는 보내달라고 하셨으므로 바로 퍼블리시
        self.lane_points_pub.publish(self.lane_points_msg)

def main():
    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
