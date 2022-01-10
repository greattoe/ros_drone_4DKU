#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from std_msgs.msg import String
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from selenium import webdriver

USE_SPHINX = bool(int(sys.argv[1]))
'''
    GPS for center of map  (  37.557953,  126.999105 )  동국대 중앙도서관
    Parot-Sphinx start GPS (  48.878900,    2.367780 )
    diffrence              ( -11.310947, +124.631325 )
OFFSET_LAT = -11.320947
OFFSET_LON = 124.631325
'''
'''
    GPS for center of map  (  37.4089445,  126.691189833 )  인천 인력개발원
    Parot-Sphinx start GPS (  48.8789000,    2.367780000 )
    diffrence              ( -11.4699555, +124.323409833 )
'''
OFFSET_LAT = -11.469955500
OFFSET_LON = 124.323409833

drv = webdriver.Chrome(executable_path="/home/gnd0/chromedriver")

def get_gps_cb(msg):    
    if USE_SPHINX is True:
        latitude  = msg.latitude  + OFFSET_LAT
        longitude = msg.longitude + OFFSET_LON
    else:
        latitude  = msg.latitude; longitude = msg.longitude        
    print("latitude = %s, longitude = %s" %(latitude, longitude))
    drv.execute_script("update_gps(%s, %s)" %(str(latitude), str(longitude)))

if __name__ == '__main__':
    drv.get('http://localhost:8080')
    rospy.sleep(3)
    rospy.init_node("bebop_gps_location", anonymous=True)
    rospy.Subscriber("bebop/states/ardrone3/PilotingState/PositionChanged", 
                     Ardrone3PilotingStatePositionChanged,
                     get_gps_cb)
    rospy.spin()
