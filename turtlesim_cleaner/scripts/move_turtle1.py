#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import radians, sqrt

LIN_SPD = 1.0
ANG_SPD = radians(90)

class MoveTurtle:

    def __init__(self):
        rospy.init_node('move_turtle', anonymous=True)
        self.pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('turtle1/pose', Pose, self.update_pose)
        
        self.tw   = Twist()
        self.pose_now = self.pose_org = Pose()
        #self.rate = rospy.Rate(10)

    def update_pose(self, msg):
        self.pose_now   = msg
        print "x=%s\ty=%s\ttheta=%s" %(self.pose_now.x, self.pose_now.y, self.pose_now.theta)
        
    def get_origin(self):
        self.pose_org = self.pose_now
    
    def elapsed_distance(self):
        return sqrt(pow((self.pose_now.x-self.pose_org.x),2)+pow((self.pose_now.y-self.pose_org.y),2))
    
    def elapsed_angle(self):
        return abs(self.pose_now.theta-self.pose_org.theta)
        
    def straight(self, dist):
        if dist < 0:
            self.tw.linear.x = -LIN_SPD
        else:
            self.tw.linear.x =  LIN_SPD
            
        self.get_origin()
        
        while self.elapsed_distance() < abs(dist):
            self.pub.publish(self.tw)
        self.tw.linear.x = 0
        self.pub.publish(self.tw)        
        
    def rotate(self, angle):
        if angle < 0:
            self.tw.angular.z = -ANG_SPD
        else:
            self.tw.angular.z =  ANG_SPD
            
        self.get_origin()
        #                                       0.9919999790346809010:
        while self.elapsed_angle() < abs(angle)*0.9919999790346809009:
            self.pub.publish(self.tw) #         0.9919999790346809008:
        self.tw.angular.z = 0
        self.pub.publish(self.tw)

if __name__ == '__main__':
    try:
        mt = MoveTurtle()
        angle = radians(input("input angle to rotate(deg): "))
        mt.rotate(angle)
        dist  = float(input("input distance to move(m): "))
        mt.straight(dist)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
