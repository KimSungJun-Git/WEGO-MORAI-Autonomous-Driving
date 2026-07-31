#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class Lane_sub:
    def __init__(self) :
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.cam_CB)
        self.speed_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1)
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
        self.speed_msg = Float64()
        self.steer_msg = Float64()

        # --- 정지선 모드 상태 ---
        self.stopline_mode = False
        self.stopline_hold = 0
        self.stopline_hold_frames = 15     # 정지선 감지 후 유지할 프레임 수
        self.ref_center_x = None           # 정지선 모드에서 사용할 가상 기준 center_x
        self.virtual_center_ratio = 0.55   # 화면폭 * 0.55 (우측 치우침 예시)

    def cam_CB(self,msg):
        # === 디코드 & 크기 ===
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.y,self.x = self.img.shape[0:2]

        # === 전처리 ===
        self.filtered   = self.filter_colors(self.img)
        self.warped_img = self.perspective_transform(self.filtered, self.x, self.y)
        self.bin_img    = self.get_binary_image(self.warped_img)

        # === 차선 중심 계산(일반 모드용) ===
        self.histogram     = np.sum(self.bin_img,axis=0)
        self.center_index  = self.steer_center_index(self.histogram, self.x)

        # === 정지선 감지 & 모드 전환 ===
        found = self.detect_stop_line()
        if found:
            # 정지선 모드 진입: 가상 기준 설정 및 유지 카운트 리셋
            self.stopline_mode = True
            self.stopline_hold = self.stopline_hold_frames
            self.ref_center_x  = int(self.x * self.virtual_center_ratio)
            rospy.loginfo("[STOPLINE] detected → virtual center_x=%d (ratio=%.2f)",
                          self.ref_center_x, self.virtual_center_ratio)

        # 정지선 모드 유지/해제
        if self.stopline_mode:
            self.stopline_hold -= 1
            if self.stopline_hold <= 0:
                self.stopline_mode = False
                self.ref_center_x  = None
                rospy.loginfo("[STOPLINE] release")

        # === 제어 ===
        self.control_vehicle()

        # === 디버그 뷰 ===
        cv2.imshow("img", self.img)
        cv2.imshow("warped_img", self.warped_img)
        cv2.waitKey(1)

    def filter_colors(self, img):
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([15,128,0])
        yellow_upper = np.array([40,255,255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        white_lower  = np.array([0,0,192])
        white_upper  = np.array([179,64,255])
        white_range  = cv2.inRange(img_hsv, white_lower, white_upper)
        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_img = cv2.bitwise_and(img,img,mask=combined_range)
        return filtered_img

    def perspective_transform(self, img, x, y):
        # 입력 영상 기준 4점 → 탑뷰
        src_point1 = [0,420]
        src_point2 = [275,260]
        src_point3 = [self.x - 275,260]
        src_point4 = [self.x,420]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])

        dst_point1 = [self.x//8,480]
        dst_point2 = [self.x//8,0]
        dst_point3 = [self.x//8*7,0]
        dst_point4 = [self.x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])

        matrix = cv2.getPerspectiveTransform(src_points,dst_points)
        return cv2.warpPerspective(img, matrix, (int(x), int(y)))

    def get_binary_image(self, img):
        grayed_img = cv2.cvtColor(self.warped_img,cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 100] = 255
        return bin_img

    def steer_center_index(self, histogram, x):
        left_hist  = histogram[0:x//2]
        right_hist = histogram[x//2:]

        self.left_indices  = np.where(left_hist  > 50)[0]
        self.right_indices = np.where(right_hist > 50)[0] + x//2
        self.indices       = np.where(histogram  > 50)[0]

        try:
            if len(self.left_indices) != 0 and len(self.right_indices) != 0:
                center_index = (self.indices[0] + self.indices[-1]) // 2
                # print("both line", center_index)
            elif len(self.left_indices) != 0:
                center_index = (self.left_indices[0] + self.left_indices[-1]) // 2
                # print("left_line", center_index)
            elif len(self.right_indices) != 0:
                center_index = (self.right_indices[0] + self.right_indices[-1]) // 2
                # print("right_line", center_index)
            else:
                center_index = x // 2
                # print("no line")
        except Exception as e:
            rospy.logerr(f"Error calculating center_index: {e}")
            center_index = x // 2
        return center_index

    def detect_stop_line(self):
        """워프 결과 하단 밴드에서 수평 긴 선(정지선) 감지 → bool"""
        band_h = max(20, int(self.y * 0.20))
        band   = self.bin_img[self.y - band_h : self.y, :]
        edges  = cv2.Canny(band, 50, 150)
        lines  = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=80,
                                 minLineLength=int(self.x * 0.45), maxLineGap=10)
        if lines is not None:
            for x1, y1, x2, y2 in lines[:,0,:]:
                if abs(y2 - y1) <= 3 and abs(x2 - x1) >= int(self.x * 0.45):
                    # 디버그로 워프 이미지에 표시(원본 밴드 좌표 보정)
                    cv2.line(self.warped_img,
                             (x1, y1 + self.y - band_h),
                             (x2, y2 + self.y - band_h),
                             (0,255,0), 3)
                    return True
        return False

    def control_vehicle(self):
        """정지선 모드면 ref_center_x 사용, 아니면 일반 center_index 사용"""
        standard_line     = self.x // 2
        degree_per_pixel  = 1.61 / self.x

        if self.stopline_mode and self.ref_center_x is not None:
            used_center = self.ref_center_x
        else:
            used_center = self.center_index

        steer_offset = (used_center - standard_line) * degree_per_pixel
        steer = 0.56 + steer_offset
        steer = float(max(0.0, min(1.0, steer)))  # 안전 클리핑

        self.steer_msg.data = steer
        self.speed_msg.data = 2000
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

def main():
    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__" :
    main()
