#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_interfaces.srv import Live
from gs_interfaces.msg import SimpleBatteryState, Orientation
from rospy import ServiceProxy, Subscriber
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class SensorManager():
    def __power_callback(self,data):
        self.__battery_state = data

    def __gyro_callback(self,data):
        self.__gyro = data
    
    def __accel_callback(self,data):
        self.__accel = data

    def __orientation_callback(self,data):
        self.__orientation = data

    def __altitude_callback(self,data):
        self.__altitude = data.data

    def __init__(self, namespace = ""):
        if namespace != "":
            namespace += "/"
        self.error_number = -255.
        self.__battery_state = SimpleBatteryState()
        self.__gyro = Point()
        self.__accel = Point()
        self.__orientation = Orientation()
        self.__altitude = 0.0
        rospy.wait_for_service(f"{namespace}geoscan/alive")
        self.__alive = ServiceProxy(f"{namespace}geoscan/alive", Live)
        self.__gyro_subscriber = Subscriber(f"{namespace}geoscan/sensors/gyro", Point, self.__gyro_callback)
        self.__accel_subscriber = Subscriber(f"{namespace}geoscan/sensors/accel", Point, self.__accel_callback)
        self.__orientation_subscriber = Subscriber(f"{namespace}geoscan/sensors/orientation", Orientation, self.__orientation_callback)
        self.__altitude_subscriber = Subscriber(f"{namespace}geoscan/sensors/altitude", Float32, self.__altitude_callback)
        self.__power_subscriber = Subscriber(f"{namespace}geoscan/battery_state", SimpleBatteryState, self.__power_callback)
        rospy.wait_for_message(f"{namespace}geoscan/sensors/gyro", Point)
        rospy.wait_for_message(f"{namespace}geoscan/sensors/accel", Point)
        rospy.wait_for_message(f"{namespace}geoscan/sensors/orientation", Orientation)
        rospy.wait_for_message(f"{namespace}geoscan/sensors/altitude", Float32)
        rospy.wait_for_message(f"{namespace}geoscan/battery_state", SimpleBatteryState)

    def gyro(self):
        if self.__alive().status:
            return self.__gyro.x,self.__gyro.y,self.__gyro.z
        else:
            rospy.logwarn("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number

    def accel(self):
        if self.__alive().status:
            return self.__accel.x,self.__accel.y,self.__accel.z
        else:
            rospy.logwarn("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number

    def orientation(self):
        if self.__alive().status:
            return self.__orientation.roll,self.__orientation.pitch,self.__orientation.azimuth
        else:
            rospy.logwarn("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number

    def altitude(self):
        if self.__alive().status:
            return self.__altitude
        else:
            rospy.logwarn("Wait, connecting to flight controller")
            return self.error_number       

    def power(self):
        if self.__alive().status:
            return self.__battery_state.charge,self.__battery_state.header.stamp.secs
        else:
            rospy.logwarn("Wait, connecting to flight controller")
            return self.error_number,self.error_number