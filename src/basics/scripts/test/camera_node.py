#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import  Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.center_pub = rospy.Publisher("/driving_center", Int32, queue_size = 3)
        self.bridge = CvBridge()
        self.fixed_center = 320
        self.yellow_lower = np.array([15, 128, 0])
        self.yellow_upper = np.array([40, 255, 255])
        self.white_lower = np.array([0, 0, 192])
        self.white_upper = np.array([179, 64, 255])
        rospy.Timer(rospy.Duration(1.0 / 10), self.timerCB)

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.y, self.x = img.shape[:2]
        filtered_img = self.hsv(img)
        warped_img = self.perspective_transform(filtered_img)
        bin_img = self.binary(warped_img)
        left_indices, right_indices = self.histogram(bin_img)
        center_index = self.judgement(left_indices, right_indices)
        self.publish(center_index)
        
        cv2.imshow("Original", img)
        cv2.imshow("Warped", warped_img)
        cv2.waitKey(1)

    def hsv(self, data):
        src_img = data
        img_hsv = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)
        yellow_range = cv2.inRange(img_hsv, self.yellow_lower, self.yellow_upper)
        white_range = cv2.inRange(img_hsv, self.white_lower, self.white_upper)
        combined_range = cv2.bitwise_or(yellow_range, white_range)
        filtered_img = cv2.bitwise_and(src_img, src_img, mask=combined_range)
        return filtered_img

    def perspective_transform(self, data):
        filtered_img = data
        src_points = np.float32([[0, 460], [225, 315], [415, 315], [640, 460]])
        dst_points = np.float32([[82, 480], [118, 0], [540, 0], [575, 480]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(filtered_img, matrix, (self.x, self.y))
        return warped_img
    
    def binary(self, data):
        warped_img = data
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 50] = 255
        return bin_img

    def histogram(self, data):
        bin_img = data
        histogram = np.sum(bin_img, axis=0)
        left_hist = histogram[0:self.fixed_center]
        right_hist = histogram[self.fixed_center:]
        left_indices = np.where(left_hist > 5)[0]
        right_indices = np.where(right_hist > 5)[0] + self.fixed_center
        return left_indices, right_indices

    def judgement(self, l_data, r_data):
        left_indices = l_data
        right_indices = r_data
        center_index = self.fixed_center  # 기본값

        # 디버그 출력 (원하면 유지)
        if left_indices.size > 0 or right_indices.size > 0:
            print(left_indices, right_indices)
        print(len(left_indices), len(right_indices))

        # --- 양쪽 차선 ---
        if len(left_indices) > 0 and len(right_indices) > 0:
            if len(left_indices) > 70:
                center_index = int(0.5 * right_indices[-1] + 40)
                rospy.loginfo("stop line detected")
            elif len(right_indices) > 60:
                center_index = int(0.5 * right_indices[-1] + 40)
                rospy.loginfo("right line detected")
            else:
                center_index = int((left_indices[0] + right_indices[-1]) / 2)
                rospy.loginfo("Both lines detected")
            return center_index

        # --- 오른쪽만 ---
        if len(right_indices) > 0:
            center_index = int(0.5 * right_indices[-1] + 40)
            rospy.loginfo("right lines only")
            return center_index

        # --- 왼쪽만 ---
        if len(left_indices) > 0:
            # 간단한 휴리스틱: 왼쪽 가장자리와 기준 센터의 중간값
            center_index = int((left_indices[0] + self.fixed_center) / 2)
            rospy.loginfo("left lines only (heuristic)")
            return center_index

        # --- 아무것도 없음 ---
        rospy.logwarn("No lines detected, using fixed center")
        return center_index


        
        
        
        
        #     center_index = (left_indices[0] + right_indices[-1]) // 2
        #     rospy.loginfo(" Both lines detected")
        #     print(center_index)
        #     return center_index
        
        # elif len(left_indices) > 90:
        #     center_index = ((0.5 * right_indices[-1]) + 55)
        #     rospy.loginfo(" right lines only ")
        #     print(center_index)
        #     return center_index
        
        # elif left_indices[0] == 0:
        #     center_index = (left_indices[120] + right_indices[-1]) // 2
        #     print(center_index)
        #     return center_index

    def publish(self, data):
        # None/NaN 방어
        if data is None or (isinstance(data, float) and not np.isfinite(data)):
            rospy.logwarn("center_index is None/NaN -> using last_center")
            center = getattr(self, "last_center", self.fixed_center)
        else:
            center = int(data)
    
        # 마지막 정상값 갱신
        self.last_center = center
    
        msg = Int32()
        msg.data = center
        self.center_pub.publish(msg)
        
    def timerCB(self, data):
        pass

def main():

    try:
        lane_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()