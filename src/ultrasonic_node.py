#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from rospy import Publisher
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from rospy import sleep
from time import time

class Ultrasonic:
    def __init__(self, trig, echo, err):
        self.TRIG = trig
        self.ECHO = echo
        self.sleep_time = 0.0051
        self.err = err
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        
    def getDistance(self):
        try:
            GPIO.output(self.TRIG, False)
            sleep(0.0001)
            GPIO.output(self.TRIG, True)
            sleep(0.005)
            GPIO.output(self.TRIG, False)
            i=  0
            while GPIO.input(self.ECHO) == 0:
                start = rospy.Time().to_sec()
                i += 1
                if i > self.err:
                    return float("inf")
                
            i = 0
            while GPIO.input(self.ECHO) == 1:
                end = rospy.Time().to_sec()
                i += 1
                if i > self.err:
                    return float("inf")

            distance = round((end - start) * 17150, 2)
            
            return distance
        except:
            return float("inf")

if __name__ == "__main__":
    rospy.init_node("ultrasonic_node")

    echo_port = int(rospy.get_param(rospy.search_param("echo")))
    trig_port = int(rospy.get_param(rospy.search_param("trig")))
    try:
        err = int(rospy.get_param(rospy.search_param("err")))
    except:
        err = 10000

    ultrasonic_publisher = Publisher(f"ultrasonic_sensor/trig_{trig_port}_echo_{echo_port}", Float32, queue_size = 10)
    ultrasonic = Ultrasonic(trig_port, echo_port, err)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        distance = ultrasonic.getDistance()
        ultrasonic_publisher.publish(Float32(distance))
        rate.sleep()