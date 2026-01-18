#! /usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist

class CarrotFollower :
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # turtle2/cmd_vel 속도를 받는 토픽으로 여기서 publish하는 데이터가 이곳으로 이동하여 turtle2를 제어 각도와 속도 제공
        self.pub = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size = 10)
    
    def follow(self):
        try:
            # turtle2와 carrot사이 거리와 회전 축을 현 시점으로 계산
            trans = self.tfBuffer.lookup_transform('turtle2', 'carrot', rospy.Time.now(), timeout = rospy.Duration(1.0))
            
            msg = Twist()
            #sqrt 제곱근 구하는 함수
            # x좌표 즉 거리를 계산 한값 저장
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x**2 + 
                                            trans.transform.translation.y**2)
            # z축 회전 방향을 저장
            msg.angular.z = 4 * math.atan2(trans.transform.translation.y, 
                                            trans.transform.translation.x)
            # 이 정보를 turtle2에게 publish
            self.pub.publish(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
            print('except')
        
def main():
    rospy.init_node('carrot_follower')
    cf = CarrotFollower()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cf.follow()
        rate.sleep()
        
        
if __name__ == "__main__":
    main()