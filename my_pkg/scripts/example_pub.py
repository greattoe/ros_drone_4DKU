#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 1행은 셔뱅(Shebang)으로, 이 스크립트 해석기의 위치를 지정한다. 모든 파이썬으로 작성된
# ROS 노드는 반드시 이 셔뱅으로 시작해야 한다.
# 2행은 한글 지원

import rospy                    # roscpp 코드의 "#include <ros.h>"에 해당하는 구문
from std_msgs.msg import String # ROS 표준 메세지의 String 모듈 import

def simple_pub():               # simple_pub() 함수 정의 시작
    # 'sample_pub' 노드 초기화 
    rospy.init_node('sample_pub', anonymous=True)
    # String 형식 토픽 'hello'를 발행하는 퍼블리셔 'pub' 선언
    pub = rospy.Publisher('hello', String, queue_size=10)
    # 1초당 10회의 빈도로 토픽을 발행하기 위한 rate 객체 선언 
    rate = rospy.Rate(10) # 10hz
    
    # rospy가 종료되지 않았으면 반복할 루프
    while not rospy.is_shutdown(): # roscpp 코드의 "while(ros::ok())"에 해당하는 구문
        # "hello~ " 문자열 뒤에 현재 시간을 덧붙인 문자열을 String 변수 str에 치환 
        str = "hello~ %s" % rospy.get_time()
        # time stamp가 표시되는 화면출력으로 str 출력
        #rospy.loginfo(str)
        # 퍼블리셔 'pub'으로 'str'의 내용을 토픽명 'hello'로 발행
        pub.publish(str)
        # 루프 시작 부터 1/10초가 지날 때까지 시간지연(토픽 발행 빈도 10회/초)
        rate.sleep()

if __name__ == '__main__':  # 모듈명이 저장되는 전역변수 __name__에 저장된 값이 '__main__'이면
    try:                    # 뒤에 나오는 예외처리(except ... :)를 고려한 실행 구간 시작
        simple_pub()        # simple_pub() 함수 호출
    except rospy.ROSInterruptException: # ROS 인터럽트 예외 발생시
        print "Program terminated"      # 프로그램 종료 메세지 화면출력
