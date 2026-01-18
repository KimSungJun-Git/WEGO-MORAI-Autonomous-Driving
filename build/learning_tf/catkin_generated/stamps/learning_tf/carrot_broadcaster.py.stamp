#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

class CarrotBroadcaster :
    def __init__(self):
        #tf2_ros객체 생성, TransformStamped 객체 생성
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.transformstamped = TransformStamped()
        # header와 child의 이름을 각각 할당해줌
        self.transformstamped.header.frame_id = "turtle1"
        self.transformstamped.child_frame_id = "carrot"
        
    def broadcast(self):
        # 변환 정보를 현재 시간 기준이라는 점을 명시해줌, 현재 시간을 기준으로 변화 추적이 가능해짐
        self.transformstamped.header.stamp = rospy.Time().now()
        # y좌표 및 회전 크기 초기화
        self.transformstamped.transform.translation.y = 0.5
        self.transformstamped.transform.rotation.w = 1.0
        # tf2_ros함수 sendTransform을 활용하여 transformstamped값 전달
        self.br.sendTransform(self.transformstamped)
        
def main():
    rospy.init_node('CarrotBroadcaster')
    #객체 생성
    cb = CarrotBroadcaster()
    #주기 10 Hz
    rate = rospy.Rate(10)
    #while문을 통해 함수 반복
    while not rospy.is_shutdown():
        cb.broadcast()
        rate.sleep()
        
        
if __name__ == "__main__":
    main()