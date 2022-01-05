#!/usr/bin/env python

import sys
import rospy
from turtlesim.msg import Pose
from math import degrees, radians, sin, cos, pi
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker

class MarkerPose:

    def __init__(self):    
        rospy.init_node('recog_ar_marker')        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_ar_pose)
        self.ar_pose = Pose()
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

                    self.print_ar_pose()
                    
                
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
    
        
    def print_ar_pose(self):
        print("ar_pose.x = %s, ar_pose.y = %s, ar_pose.theta = %s" \
              %(self.ar_pose.x, self.ar_pose.y, degrees(self.ar_pose.theta)))
          

if __name__ == '__main__':
    try:        
        MarkerPose()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
