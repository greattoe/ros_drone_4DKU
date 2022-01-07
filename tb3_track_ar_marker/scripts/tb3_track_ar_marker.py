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
# MAX_LIN_SPEED =  0.22 # (m/s)
# MAX_ANG_SPEED =  2.84 # (rad/s)

# make default speed of linear & angular
LIN_SPD = 0.125
ANG_SPD = 0.125

class MarkerPose:

    def __init__(self):    
        rospy.init_node('recog_ar_marker')        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_ar_pose) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.tw  = Twist()
        self.tag = AlvarMarker()
        self.tb3 = MoveTB3()
        
        self.theta  = 0.0
        self.pos_x  = 0.0
        self.pos_z  = 0.0
        
        self.ref_th = 0.0
        self.ref_d  = 0.0
        
        self._1_target_found     = False
        self._2_align_to_marker  = False
        self._3_calc_th_dist     = False
        self._4_rotate_to_front  = False
        self._5_move_to_front    = False
        self._6_rotate_to_marker = False
        self._7_approach_marker  = False
        
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
    
        self.tw.angular.z = ANG_SPD;    self.pub.publish(self.tw)
        
        if len(msg.markers) != 0:
            
            for msg in msg.markers:
                
                if msg.id == TARGET_ID:
                    
                    self.tw.angular.z = 0;  self.pub.publish(self.tw);  rospy.sleep(0.5)
                    
                    if self._1_target_found is False:
                        self._1_target_found = True
                        print "--- 1. target maerker found"
                    
                    self.tag = msg
                    '''                    
                    theta = self.get_theta(msg)
                    
					# make theta from -90 to 90                    
                    if   theta >  radians(270): 
                        self.theta = theta - 2 * pi            
                    elif theta < -radians(270):
                        self.theta = theta + 2 * pi
                    else:
                        self.theta = theta
                    '''
                    if  self._1_target_found is True and self._2_align_to_marker is False:
                        
                        print(self.tag.pose.pose.position.x)
                        
                        if self.tag.pose.pose.position.x < -0.025 or self.tag.pose.pose.position.x > 0.025:
                            self.tw.angular.z = ANG_SPD;    self.pub.publish(self.tw)
                        else:
                            self.tw.angular.z = 0;  self.pub.publish(self.tw);  rospy.sleep(0.5)
                            self._2_align_to_marker = True
                            print "--- 2. align to marker fineshed"
                    
                    if self._2_align_to_marker is True and self._3_calc_th_dist is False:
                        self.get_ref()
                        self._3_calc_th_dist = True
                        print "--- 3. calcurate(angle & distance) fineshed"
                    
                    if self._3_calc_th_dist is True and self._4_rotate_to_front is False:
                        self.tb3.rotate(self.ref_th)
                        self._4_rotate_to_front = True
                        print "--- 4. rotation(to front) fineshed"
        else:
            if self._4_rotate_to_front is True and self._5_move_to_front is False:
                self.tb3.straight(self.ref_d)
                    
                '''
                self._1_target_found     = False
                self._2_align_to_marker  = False
                self._3_calc_th_dist     = False
                self._4_rotate_to_front  = False
                self._5_move_to_front    = False
                self._6_rotate_to_marker = False
                self._7_approach_marker  = False                    
                '''
                    
                
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
                
        
    def get_ref(self, msg):
        theta = self.get_theta(msg)
        
		# make theta from -90 to 90                    
        if   theta >  radians(270): 
            theta = theta - 2 * pi            
        elif theta < -radians(270):
            theta = theta + 2 * pi
        else:
            pass

        self.ref_th = theta
        self.ref_d  = msg.pose.pose.position.z * sin(self.ref_th)
          

if __name__ == '__main__':
    try:        
        mp = MarkerPose()
        mt = MoveTB3()
        '''
        print "--- step1. Searching Target Marker"
        
        mp.tw.angular.z = ANG_SPD
        while mp.target_found is False:
            print "---";    mp.pub.publish(mp.tw)
        
        mp.tw.angular.z = 0.0;  mp.pub.publish(mp.tw)
        
        print "    target marker is found!"
        '''
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
