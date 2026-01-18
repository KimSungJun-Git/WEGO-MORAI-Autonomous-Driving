#! /usr/bin/env python3

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class PubTf:
    def __init__(self):
        rospy.init_node("odom_tf_broadcast", anonymous = 10)
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        rospy.Subscriber('odom', Odometry, self.callback)
        
    def callback(self, msg):
        self.t.header.frame_id = 'odom'
        self.t.header.stamp = rospy.Time().now()
        self.t.child_frame_id = msg.child_frame_id
        self.t.transform.translation.x = msg.pose.pose.position.x
        self.t.transform.translation.y = msg.pose.pose.position.y
        self.t.transform.translation.z = msg.pose.pose.position.z
        self.t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.t)

# 메인함수에서 try except 구문과 while 구문 상황별 차이점
# 오류에 대한 예외처리를 우선하는 경우는 try except구문 사용, 코드의 반복적인 동작을 확인하고 싶은 경우 while 구문 사용
def main():
    try:
        pub_tf = PubTf()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
                
if __name__ == "__main__":
    main()