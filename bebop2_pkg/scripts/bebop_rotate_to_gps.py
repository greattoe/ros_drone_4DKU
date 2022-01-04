#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged, \
                           Ardrone3PilotingStatePositionChanged
from scipy import cos, sin, arctan2, pi
from math import degrees, radians

USE_SPHINX = bool(int(sys.argv[1]))
'''
    GPS for center of map  (  37.557953,  126.999105 )
    Parot-Sphinx start GPS (  48.878900,    2.367780 )
    diffrence              ( -11.310947, +124.631325 )
'''
OFFSET_LAT = -11.320947
OFFSET_LON = 124.631325
PI         =   3.14159265358979323846
ANG_SPD    =   0.25 * PI
'''     distance  of latitude   1(deg) = 111195.0802340(m/deg)
        distance  of longtitude 1(deg) =  88148.1011811(m/deg)
        latitude  of distance   1(m)   =      0.00000899320363721(deg/m)
        longitude of distance   1(m)   =      0.00001134454385970(deg/m)
 
        -------------+---------------------+----------------------
         Distance(m) |    latitude(deg)    |    longitude(deg)
        -------------+---------------------+----------------------
               1.0   | 0.00000899320363721 | 0.0000113445438597
              10.0   | 0.0000899320363721  | 0.000113445438597
        -------------+---------------------+----------------------
             100.0   | 0.000899320363721   | 0.00113445438597 ***
        -------------+---------------------+----------------------

|<-- 100(m)-->|<-- 100(m)-->|
           --- p8------------p1-------------p2-> 37.5588172985 
            ^   | .-45        |0          . |   (37.55791797814083+0.000899320363721)
            |   |   .         |         . 45|
           100  |     .       |       .     |
           (m)  |       .     |     .       |
            |   |         .   |   .         |
            v   |-90        . | .           |
           --- p7------------p0-------------p3-> 37.55791797814083
            ^   |           . | .         90|
            |   |         .   |   .         |
           100  |       .     |     .       |
           (m)  |     .       |       .     |
            |   -135.         |         .   |
            v   | .           |       135 . |
           --- p6------------p5-------------p4-> 37.5570186578
                |             |             |   (37.55791797814083-0.000899320363721)
                v             v             v
       126.997871304  126.9990057580825  127.000140212
          ^                                 ^
          |                                 |
   (126.9990057580825-0.00113445438597)     |
                                     (126.9990057580825+0.00113445438597)
     
p0 = (37.55791797814083, 126.9990057580825)
        
        p0 = (37.5579179781, 126.999005758)
        p1 = (37.5588172985, 126.999005758);   p5 = (37.5570186578, 126.999005758)
        p2 = (37.5588172985, 127.000140212);   p6 = (37.5570186578, 126.997871304)
        p3 = (37.5579179781, 127.000140212);   p7 = (37.5579179781, 126.997871304)
        p4 = (37.5570186578, 127.000140212);   p8 = (37.5588172985, 126.997871304)
'''
class RotateByGPS:
    
    def __init__(self):
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged,
                         self.cb_get_atti)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged',
                         Ardrone3PilotingStatePositionChanged,
                         self.cb_get_gps)
                         
        self.atti_now  = 0.0
        self.atti_tmp  = 0.0
        
        self.use_tmp   = False
        
        self.lati_now = 500.0
        self.long_now = 500.0

    def cb_get_gps(self, msg):
        
        if USE_SPHINX is True:
            self.lati_now = msg.latitude  + OFFSET_LAT
            self.long_now = msg.longitude + OFFSET_LON
        else:
            self.lati_now = msg.latitude
            self.long_now = msg.longitude

    def cb_get_atti(self, msg):
    
        self.atti_now = msg.yaw
        
        if   msg.yaw < 0:
            self.atti_tmp = msg.yaw + PI 
        else:
            self.atti_tmp = msg.yaw - PI
    
    def get_atti(self):
        if self.use_tmp == True:
            return self.atti_tmp
        else:
            return self.atti_now
    
    def get_bearing(self, lat1, lon1, lat2, lon2):
    
        Lat1,  Lon1 = radians(lat1), radians(lon1) 
        Lat2,  Lon2 = radians(lat2), radians(lon2) 
        
        y = sin(Lon2-Lon1) * cos(Lat2) 
        x = cos(Lat1) * sin(Lat2) - sin(Lat1) * cos(Lat2) * cos(Lon2-Lon1) 
        
        return arctan2(y, x)
        
    def get_gps_now(self):
        return self.lati_now, self.long_now     
    
    def rotate(self, lat2, lon2, speed):
        
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        tw  = Twist()
        
        lat1, lon1 = self.get_gps_now()
        
        target  = self.get_bearing(lat1, lon1, lat2, lon2)
        
        current = self.atti_now;        angle = abs(target-current)
        
        print "target:%s, current:%s, angle:%s" %(degrees(target), degrees(current), degrees(angle))
        
        if angle > pi:  #   if angle > radians(180):
            print "---"
            self.use_tmp = True            
            if   target > 0.0:
                target = target - pi
            elif target < 0.0:
                target = target + pi
            else:   pass
            current = self.get_atti();  angle = abs(target - current)
            print "target:%s, current:%s, angle:%s" %(degrees(target), degrees(current), degrees(angle))
        else:           #   if angle > radians(180):
            self.use_tmp = False        
        
        print "start from: %s" %(degrees(self.atti_now))
            
        if   target > current:    # cw, -angular.z
            tw.angular.z = -speed
            
            if angle > radians(50):
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
                pub.publish(tw);    current = self.get_atti()
        elif target < current:    # ccw,  angular.z
            tw.angular.z =  speed
           if angle > radians(50):
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
                pub.publish(tw);    current = self.get_atti()
        else:   pass
        
        tw.angular.z = 0.0; pub.publish(tw); rospy.sleep(3.0)
        print "stop to   : %s" %(degrees(self.atti_now))
        
            
if __name__ == '__main__':
    
    rospy.init_node('bebop_rotate_to_gps', anonymous = True)
    rbg = RotateByGPS()
    
    try:
        while not rospy.is_shutdown():
            p2_lati_deg = float(input("input target latitude : "))
            p2_long_deg = float(input("input target longitude: "))
            
            rbg.rotate(p2_lati_deg, p2_long_deg)
            
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
