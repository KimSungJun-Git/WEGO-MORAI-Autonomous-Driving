#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
import numpy as np

class controller :
    def __init__(self):
        rospy.init_node("control_v")
        rospy.Subscriber("/driving_center", Int32, self.steer)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        # PID 제어 상수
        self.Kp = 0.0001 # 비례 상수
        self.Ki = 0.00001 # 적분 상수 (차량의 편향 보정)
        self.Kd = 0.0   # 미분 상수 (급격한 변화 억제)

        self.i_error = 0
        self.p_error = 0
        self.set_point = 320
        
    def steer(self, data):
        center_index = data.data
        
        if center_index is not None:
            if center_index == 0:
                self.stop_line()
                
            elif center_index:
                self.go_straight(center_index)
            else:
                pass
        else:
            pass

    def stop_line(self):
        start_time = rospy.get_time()
        if (rospy.get_time() - start_time < 2.0):
            self.speed_msg.data = 2500
            self.speed_pub.publish(self.speed_msg)
        else:
            pass
        
    def go_straight(self, data):
        center_index = data
        
        error = self.set_point - center_index
        self.i_error += error
        derivative = error - self.p_error
        
        steering_angle = (self.Kp * error) + (self.Ki * self.i_error) + (self.Kd * derivative)
        self.p_error = error
        print(steering_angle)
        
        self.publish(steering_angle)
        
    def publish(self, data):
        steering_angle = data
        steer = 0.5 - steering_angle
        steer_cali = round(steer, 4)
        print(steer_cali)
        self.steer_msg.data = steer_cali
        self.speed_msg.data = 2500
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)
        pass
        
def main():
    try:
        ctrl = controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == "__main__":
    main()