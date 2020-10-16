#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_interfaces.srv import LpsVel,LpsYaw,Gyro,Accel,Orientation,Live,Altitude
from gs_interfaces.msg import SimpleBatteryState
from rospy import ServiceProxy,Subscriber
from std_msgs.msg import Bool

class SensorManager():
    def __power_callback(self,data):
        self.__battery_state=data

    def __init__(self):
        self.error_number=-255.
        self.__battery_state=SimpleBatteryState()
        rospy.wait_for_service("geoscan/alive")
        rospy.wait_for_service("geoscan/sensors/lpsvel_service")
        rospy.wait_for_service("geoscan/sensors/lpsyaw_service")
        rospy.wait_for_service("geoscan/sensors/gyro_service")
        rospy.wait_for_service("geoscan/sensors/accel_service")
        rospy.wait_for_service("geoscan/sensors/orientation_service")
        self.__alive=ServiceProxy("geoscan/alive",Live)
        self.__lpsvel_service=ServiceProxy("geoscan/sensors/lpsvel_service",LpsVel)
        self.__lpsyaw_service=ServiceProxy("geoscan/sensors/lpsyaw_service",LpsYaw)
        self.__gyro_service=ServiceProxy("geoscan/sensors/gyro_service",Gyro)
        self.__accel_service=ServiceProxy("geoscan/sensors/accel_service",Accel)
        self.__orientation_service=ServiceProxy("geoscan/sensors/orientation_service",Orientation)
        self.__altitude_service=ServiceProxy("geoscan/sensors/altitude_service",Altitude)
        self.__power_sub=Subscriber("geoscan/battery_state",SimpleBatteryState,self.__power_callback)

    def lpsVelocity(self):
        if(self.__alive().status):
            otv=self.__lpsvel_service()
            return otv.lps_velocity.x,otv.lps_velocity.y,otv.lps_velocity.z
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number
        
    def lpsYaw(self):
        if(self.__alive().status):
            return self.__lpsyaw_service().angle
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number

    def gyro(self):
        if(self.__alive().status):
            otv=self.__gyro_service()
            return otv.gyro.x,otv.gyro.y,otv.gyro.z
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number

    def accel(self):
        if(self.__alive().status):
            otv=self.__accel_service()
            return otv.accel.x,otv.accel.y,otv.accel.z
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number

    def orientation(self):
        if(self.__alive().status):
            otv=self.__orientation_service()
            return otv.roll,otv.pitch,otv.azimuth
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number,self.error_number,self.error_number

    def altitude(self):
        if(self.__alive().status):
            return self.__altitude_service().altitude
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number       

    def power(self):
        return self.__battery_state.current,self.__battery_state.charge,self.__battery_state.header.stamp.secs