#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist
from math import degrees, radians, sin, cos, pi
from tf.transformations import euler_from_quaternion
from tb3_move import MoveTB3

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.125
ANG_SPD = MAX_ANG_SPEED * 0.125

class MarkerPose:

    def __init__(self):    
        rospy.init_node('recog_ar_marker')        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_ar_pose) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tw  = Twist()
        
        self.dist  = 0.0
        self.theta = 0.0
        
        self.pos_x = 0.0
        
        self.dist_ref  = 0.0    # distance for move to front of marker
        self.theta_ref = 0.0    # angle for calcurate dist_ref
        
        self.target_found = False
        '''
        self.zone = 0.0
        
        self.ar_tag = AlvarMarker()
        
        self.step2_align_marker  = False
        self.step3_get_ref_value = False
        '''
        """   
                                                 ////////////| ar_marker |////////////
                y                      z         --------+---------+---------+--------
                ^  x                   ^                 |     R-0/|\R-0    R|
                | /                    |                 |       /0|0\       |
         marker |/                     | robot           |      /  |  \      |
                +------> z    x <------+                 |     /   |   \     |
                                      /                  |  dist   |  dist   |
                                     /                   |   /     |     \   |
                                    y                    |  /      |      \  |
                                                         | /       |       \0|
                                                         |/R-0    R|R    R-0\|
        pose.x = position.z                      (0 < O) x---------+---------x (0 > 0)
        pose.y = position.x              [0]roll         ^                   ^   
        theta  = euler_from_quaternion(q)[1]pitch*       |                   |
                                         [2]yaw        robot               robot
        """        
    def get_ar_pose(self, msg):
        
        if len(msg.markers) != 0: # found marker at least 1EA
            
            for msg in msg.markers:
                
                if msg.id == TARGET_ID: # found target marker                    
                    
                    self.target_found = True
                                        
                    theta = self.get_theta(msg)
                    
					# make theta from -90 to 90                    
                    if   theta >  radians(270): 
                        self.theta = theta - 2 * pi            
                    elif theta < -radians(270):
                        self.theta = theta + 2 * pi
                    else:
                        self.theta = theta
                    
                    self.dist  = msg.pose.pose.position.z
                    self.pos_x = msg.pose.pose.position.x
                    
                
    def get_theta(self, msg): 
              
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """   
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[1]
        
        # make theta from 0 to 360(deg)        
        if theta < 0:
            theta = theta + radians(360)
        if theta > 2 * pi:
            theta = theta - radians(360)
        
        return theta
                
        
    def get_ref(self):
        self.theta_ref = self.theta
        self.dist_ref  = self.pos_x * sin(self.theta_ref)
        print "theta = %s, dist = %s" %(self.theta_ref, self.dist_ref)
          

if __name__ == '__main__':
    try:        
        mp = MarkerPose()
        mt = MoveTB3()
        
        print "--- step1. Searching Target Marker"
        
        mp.tw.angular.z = ANG_SPD
        
        while mp.target_found is False:
            mp.pub.publish(mp.tw)
        
        mp.tw.angular.z = 0.0
        mp.pub.publish(mp.tw)
        
        print "    Target marker is found!"
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
