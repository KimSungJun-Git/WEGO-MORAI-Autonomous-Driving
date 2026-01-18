#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

from turtlesim.msg import Pose
import tf_conversions

from turtlesim.srv import Spawn, SpawnRequest


class TurtleTwoBroadcaster :
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.transformstamped = TransformStamped()
        self.transformstamped.header.frame_id = "world"
        self.transformstamped.child_frame_id = "turtle2"
        # turtlesim에 새 거북이를 만드는 과정
        service = rospy.ServiceProxy('/spawn', Spawn)
        # 해당 서비스가 준비될 때까지 대기
        service.wait_for_service()
        # 스폰 요청
        srv = SpawnRequest()
        # 이름
        srv.name =  'turtle2'
        # 요청 전송
        service(srv)
        rospy.Subscriber('/turtle2/pose', Pose, self.callback)
        
    def callback(self, data):
        self.transformstamped.transform.translation.x = data.x
        self.transformstamped.transform.translation.y = data.y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, data.theta)
        self.transformstamped.transform.rotation.z = q[2]
        self.transformstamped.transform.rotation.w = q[3]
        
    def broadcast(self):
        self.transformstamped.header.stamp = rospy.Time().now()
        self.br.sendTransform(self.transformstamped)
        
def main():
    rospy.init_node('turtle_two_broadcaster')
    ttb = TurtleTwoBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ttb.broadcast()
        rate.sleep()
        
        
if __name__ == "__main__":
    main()