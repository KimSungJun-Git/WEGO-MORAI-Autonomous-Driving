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
        self.center_index_pub = rospy.Publisher("/driving_center", Int32MultiArray, queue_size = 3)
        self.bridge = CvBridge()
        self.center_index_msg = Int32MultiArray()
        self.fixed_center = 320
        self.lane_width = 350
        # ✅ 우회전 시 조향각을 증폭시킬 계수 (1.0이 기본, 클수록 더 많이 꺾음)

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[:2]

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_img = cv2.bitwise_and(img, img, mask=combined_range)

        # Perspective transform
        src_points = np.float32([[0, 420], [220, 315], [420, 315], [x, 420]])
        dst_points = np.float32([[x//8, 480], [132, 0], [495, 0], [x//8*7, 480]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (x, y))

        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 50] = 255

        histogram = np.sum(bin_img, axis=0)
        left_hist = histogram[0:self.fixed_center]
        right_hist = histogram[self.fixed_center:]
        
        left_indices = np.where(left_hist > 5)[0]
        right_indices = np.where(right_hist > 5)[0] + self.fixed_center
        center_index = self.judgement(left_indices, right_indices)
        self.publish(center_index, left_indices, right_indices)

        #cv2.imshow("Original", img)
        #cv2.imshow("Warped", warped_img)
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
            #rospy.loginfo(f"▶ 양쪽 차선 감지. 차선 폭 업데이트: {self.lane_width:.1f}")
            return center_index
        elif len(left_indices) > 0 and len(right_indices) == 0:
            left_pos = left_indices[-1]
            center_index = int(left_pos + self.lane_width / 2)
            #rospy.loginfo("◀ 왼쪽 차선만 감지. 가상 중앙 주행")
            return center_index
        elif len(right_indices) > 0 and len(left_indices) == 0:
            right_pos = right_indices[0]
            center_index = int(right_pos - self.lane_width / 2)
            #rospy.loginfo("▶ 오른쪽 차선만 감지. 가상 중앙 주행")
            return center_index
        else:
            #rospy.logwarn("⚠ 차선 없음. 직진 유지")
            return center_index

    def publish(self, data, left, right):
        center_index = data
        len_left = len(left)
        len_right = len(right)
        width = None
        print(len_left, len_right)
        if len_left > 0 and len_right > 0:
            width = right[0] - left[-1]
            #print(width)
            
        c_l_r_w = [center_index, len_left, len_right, width]
        self.center_index_msg.data = c_l_r_w
        if width is not None:
            self.center_index_pub.publish(self.center_index_msg)
        # if width is not None:
        #     c_l_r_w = [center_index, len_left, len_right, width]
        #     self.center_index_msg.data = c_l_r_w
        #     self.center_index_pub.publish(self.center_index_msg)
        # else:
        #     c_l_r = [center_index, len_left, len_right]
        #     self.center_index_msg.data = c_l_r
        #     self.center_index_pub.publish(self.center_index_msg)


def main():
    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
