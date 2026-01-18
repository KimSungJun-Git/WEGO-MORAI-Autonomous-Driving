#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

from turtlesim.msg import Pose
import tf_conversions

class TurtleOneBroadcaster :
    def __init__(self):
        # 값을 지정하는 것이 아닌 능동적인 값 초기화를 위해 Static이 사라짐
        self.br = tf2_ros.TransformBroadcaster()
        self.transformstamped = TransformStamped()
        self.transformstamped.header.frame_id = "world"
        self.transformstamped.child_frame_id = "turtle1"
        # turtlesim_node로부터 토픽값을 받음, callback함수 호출
        rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        
    def callback(self, data):
        # x와 y값을 /turtle1/pose로부터 받아온 값으로 초기화
        self.transformstamped.transform.translation.x = data.x
        self.transformstamped.transform.translation.y = data.y
        # tf_conversions.transformations.quaternion_from_euler함수를 객체로 만듬 (0,0,data.theta)는 각 x,y,z축으로 회전축을 결정함 z축을 중심으로 회전함을 의미
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, data.theta)
        # 오일러 공식을 빌려와 쿼터니언 변환식은 q = (x,y,z,w)으로 오일러 공식인 z = cos()+jsin()을 사용함 
        # q = jsin(x,y,z(각 회전축)) + cos(회전각 크기) 각도는 항상 절반을 적용하는데 그 이유는 회전 연산시 
        # 수하적으로 3D회전을 공간 상에 적용하려면 쿼터니안 q와 그 복소수 켤레를 곱하게 되는데 
        # 이 과정에서 회전각이 2배가 되므로 sin(90/2) cos(90/2)와 회전각의 절반만 사용함
        # if 로봇이 z축으로 90도 회전 시 q = (x = sin(0), y = sin0 z = sin 90 w = cos90)
        #... x = 0 y = 0 z = 0.707.... w = 0.707...
        self.transformstamped.transform.rotation.x = q[0] # -> x
        self.transformstamped.transform.rotation.y = q[1] # -> y
        self.transformstamped.transform.rotation.z = q[2] # -> z
        self.transformstamped.transform.rotation.w = q[3] # -> w
        
    def broadcast(self):
        self.transformstamped.header.stamp = rospy.Time().now()
        self.br.sendTransform(self.transformstamped)
        
def main():
    rospy.init_node('turtle_one_broadcaster')
    tob = TurtleOneBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tob.broadcast()
        rate.sleep()
        
        
if __name__ == "__main__":
    main()