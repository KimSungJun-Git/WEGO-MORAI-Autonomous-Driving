#!/usr/bin/env python3
import rospy
from morai_msgs.msg import GetTrafficLightStatus

def cb(msg):
    rospy.loginfo(f"{msg.trafficLightIndex} -> {msg.trafficLightStatus}")

if __name__ == "__main__":
    rospy.init_node("tl_listener")
    rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, cb, queue_size=10)
    rospy.spin()
