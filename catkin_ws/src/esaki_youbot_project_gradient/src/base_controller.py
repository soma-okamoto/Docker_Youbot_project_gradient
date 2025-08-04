#!/usr/bin/env python3

import imp
import rospy
import time
from geometry_msgs.msg import Twist


class Test():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    def pub_x(self):
        dist = 1.5
        speed = 0.1
        target_time = dist/speed

        t = Twist()
        stop = Twist()
        t.linear.x = speed
        t.angular.z = 0

        start_time = time.time()
        end_time = time.time()

        rate = rospy.Rate(30)
        
        while end_time - start_time <= target_time:
            self.pub.publish(t)
            print(t)
            end_time = time.time()
            rate.sleep()
        
        else:
            print(stop)
            self.pub.publish(stop)
            rate.sleep()
    
    def pub_y(self):
        dist = 0.2
        speed = 0.1
        target_time = dist/speed

        t = Twist()
        stop = Twist()
        t.linear.y = -speed
        t.angular.z = 0

        start_time = time.time()
        end_time = time.time()

        rate = rospy.Rate(30)
        
        while end_time - start_time <= target_time:
            self.pub.publish(t)
            print(t)
            end_time = time.time()
            rate.sleep()
        
        else:
            print(stop)
            self.pub.publish(stop)
            rate.sleep()
            

if __name__ == '__main__':

    rospy.init_node('tcmdvel_publisher')
    test = Test()
    test.pub_x()
    #test.pub_y()
