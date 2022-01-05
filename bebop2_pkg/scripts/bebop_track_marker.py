#!/usr/bin/env python

import sys
import rospy
from turtlesim.msg import Pose
from math import degrees, radians, sin, cos, pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from bebop_move import Bebop2Move

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

LIN_SPD   = 0.125
ANG_SPD   = 0.125

TARGET_H  = 1.4

class MarkerPose:

    def __init__(self):    
        rospy.init_node('recog_ar_marker')        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_ar_pose) 
        '''       
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged',
                         Ardrone3PilotingStateAltitudeChanged,
                         self.get_alti 
                        )
        '''
        self.pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.tw  = Twist()
        self.ar_pose = self.ar_pose_ref = Pose()
        
        self.ar_tag = AlvarMarker()
        
        self.alti  = 0.0
        self.dist_ref  = 0.0 # distance to move_y
        self.theta_ref = 0.0 # theta when aligned to marker
        
        self.step1_target_found  = False
        self.step2_align_marker  = False
        self.step3_get_ref_value = False
        
        
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
                
                    self.ar_tag = msg
                    
                    #print("%s" %(self.ar_tag.pose.pose.position.x))
                    
                    self.step1_target_found = True
                    
                    #if self.step2_align_marker == False:
                    
                    theta = self.get_ar_theta(msg)
                    
					# make theta from -90 to 90
                    if   theta >  radians(270): 
                        self.ar_pose.theta = theta - 2 * pi            
                    elif theta < -radians(270):
                        self.ar_pose.theta = theta + 2 * pi
                    else:
                        self.ar_pose.theta = theta

                    self.ar_pose.x = msg.pose.pose.position.z
                    self.ar_pose.y = msg.pose.pose.position.x
                    
                    #if self.step2_align_marker is False:
                        
                    
                    #self.print_ar_pose()
                    
                
    def get_ar_theta(self, msg): 
              
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
             
        euler = euler_from_quaternion(q)
        theta = euler[1]
        
        # make theta from 0 to 360(deg)
        if theta < 0:
            theta = theta + radians(360)
        if theta > 2 * pi:
            theta = theta - radians(360)

        return theta
    
    
    def get_alti(self, msg):
        self.alti = msg.altitude
        
        
    def get_ref(self):
        self.theta_ref = self.ar_pose.theta
        self.dist_ref  = self.ar_tag.pose.pose.position.z * sin(self.theta_ref)
    
        
    def print_ar_pose(self):
        print("ar_pose.x = %s, ar_pose.y = %s, ar_pose.theta = %s" \
              %(self.ar_pose.x, self.ar_pose.y, degrees(self.ar_pose.theta)))
          

if __name__ == '__main__':
    try:        
        mp  = MarkerPose()
        bb2 = Bebop2Move()
        '''
        while mp.alti < TARGET_H:
            mp.tw.linear.z = LIN_SPD
            mp.pub.publish(mp.tw)
        
        mp.tw.linear.z = 0.0
        mp.pub.publish(mp.tw)        
        '''
        mp.tw.angular.z = ANG_SPD
        
        while mp.step1_target_found == False:
            mp.pub.publish(mp.tw)
        
        mp.tw.angular.z = 0.0
        mp.pub.publish(mp.tw)
        
        mp.tw.angular.z = ANG_SPD
        while mp.ar_tag.pose.pose.position.x < -0.25 or mp.ar_tag.pose.pose.position.x > 0.25:
            mp.pub.publish(mp.tw)
        
        mp.tw.angular.z = 0.0
        mp.pub.publish(mp.tw)
        
        mp.get_ref()
        
        if mp.theta_ref >= 0:
            bb2.rotate(-mp.theta_ref * 0.75, 0.05)
        else:
            bb2.rotate( mp.theta_ref * 0.75, 0.05)
        
        if mp.theta_ref >= 0:
            bb2.move_y( mp.dist_ref * 1.25, 0.025)
        else:
            bb2.move_y(-mp.dist_ref * 1.25, 0.025)
        
        bb2.move_x(0.4, 0.05)
        
        bb2.landing()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
