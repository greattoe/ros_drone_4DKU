#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from math import degrees, radians, pi
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged

class RotateByAtti:
    
    def __init__(self):
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged,
                         self.cb_get_atti)
        self.atti_now  = 0.0
        self.atti_tmp  = 0.0
        
        self.use_tmp   = False

    def cb_get_atti(self, msg):    
        self.atti_now = msg.yaw        
        if   msg.yaw < 0:
            self.atti_tmp = msg.yaw + pi 
        elif msg.yaw > 0:
            self.atti_tmp = msg.yaw - pi
        else:
            self.atti_tmp = 0.0
    
    def get_atti(self):
        if self.use_tmp == True:
            return self.atti_tmp
        else:
            return self.atti_now
        
    
    def rotate(self, angle, speed):
        pb  = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        tw  = Twist()
        
        current = self.atti_now
        
        if angle < 0:
            target  = current + abs(angle)
        else:
            target  = current - abs(angle)
            
        if abs(target) > pi:
            self.use_tmp = True               
            if target  < 0:
                target  = target + pi
            else:
                target  = target - pi
            current = self.get_atti();  angle = abs(target - current)            
        else:
            self.use_tmp = False
        
        print "start rotate from: %s" %(degrees(self.atti_now))
                
        if   target > current:    # cw, -angular.z
            
            tw.angular.z = -speed
            
            if   angle > radians(50):
                target = target - radians(5)
            elif angle > radians(20): 
                target = target - radians(10)
            else:
                tw.angular.z = -0.1125
                
            while target > current:
                if abs(tw.angular.z) > 0.125:
                    tw.angular.z = -speed * abs(target - current) / angle
                else:
                    tw.angular.z = -0.125
                current = self.get_atti();  pb.publish(tw)
                
        elif target < current:    # ccw,  angular.z            
            
            tw.angular.z =  speed
            
            if   angle > radians(50):
                target = target + radians(5)
            elif angle > radians(20): 
                target = target + radians(10)
            else:
                tw.angular.z =  0.1125
                
            while target < current:
                if abs(tw.angular.z) > 0.125:
                    tw.angular.z =  speed * abs(target - current) / angle
                else:
                    tw.angular.z =  0.125
                current = self.get_atti();  pb.publish(tw)
                
        else:   pass
        
        tw.angular.z =  0.0;    pb.publish(tw); rospy.sleep(3.0)
        print "stop rotate to   : %s" %(degrees(self.atti_now)) 
        
        
if __name__ == '__main__':
    
    rospy.init_node('bb2_rotate_by_atti', anonymous = True)
    rba = RotateByAtti()
    
    try:
        while not rospy.is_shutdown():
            angle   = radians(float(input("input angle(deg)   to rotate: ")))
            speed   = radians(float(input("input speed(deg/s) to rotate: ")))
            rba.rotate(angle, speed)            
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
