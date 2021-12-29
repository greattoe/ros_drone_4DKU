#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def cb_func(msg): 
    rospy.loginfo(rospy.get_caller_id() + 'subscribed message: %s', msg.data)

def simple_sub():
    rospy.init_node('sample_sub') 
    rospy.Subscriber('hello', String, cb_func)
    rospy.spin()
    
if __name__ == '__main__':      # 인터프리터 전역변수 __name__ 의 값이 '__main__' 이면
    simple_sub()                # 'simple_sub()' 함수 호출
