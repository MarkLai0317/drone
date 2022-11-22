#!/usr/bin/env python
import rospy
import time
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt8, Float64MultiArray
from time import sleep
from tello_driver.msg import TelloStatus

class TelloState:
    
    def __init__(self):
    
        self.height = 0.0
        self.temperature_height_m = 0.0
        self.battery = 100.0
        self.is_flying = False
        self.fly_mode = 999

        self.rec_count = 0
        self.target_x = [-1, -1]
        self.target_y = [-1, -1]
        self.canPass = [-1, -1]

        self.tag_id = -1

class TelloController:

    def move(self, twist, limitTime):
        limitTime = limitTime * 1000
        startTime = int(round(time.time()*1000))
        rate = rospy.Rate(10)
        # print "MOVE~~~~"
        pub_move = rospy.Publisher("/tello/cmd_vel", Twist, queue_size = 10)

        while not rospy.is_shutdown():
            connections = pub_move.get_num_connections()
            if connections > 0:
                endTime = int(round(time.time()*1000))
                if endTime - startTime < limitTime:
                    pub_move.publish(twist)

                else:
                    #pub_move.publish(Twist())
                    break
                rate.sleep() 
    
    def rotate(self, vel_msg, clockwise=False, angle=90,speed = 60):
        PI = 3.1415926535897
        velocity_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        

        # Receiveing the user's input
        

        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #We wont use linear components
        vel_msg.linear.x=1
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)


        #Forcing our robot to stop
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

    def emergency(self):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/emergency", Empty, queue_size=10) 
        
        while not rospy.is_shutdown():
            cons = puber.get_num_connections()
          
            if cons > 0:
                puber.publish(Empty())
                rate.sleep()
                break
        
    def takeoff(self):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/takeoff", Empty, queue_size=10) 
        
        while not rospy.is_shutdown():
            cons = puber.get_num_connections()
          
            if cons > 0:
                puber.publish(Empty())
                rate.sleep()
                break
            
    def land(self):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/land", Empty, queue_size=10) 
        
        while not rospy.is_shutdown():
          cons = puber.get_num_connections()
          
          if cons > 0:
            puber.publish(Empty())
            rate.sleep()
            break
    
    def flip(self, i):
        rate = rospy.Rate(10)
        
        puber = rospy.Publisher("/tello/flip", UInt8, queue_size=10) 
        
        while not rospy.is_shutdown():
            cons = puber.get_num_connections()
          
            if cons > 0:
                puber.publish(UInt8(data=i))
                rate.sleep()
                break
    
class Tello_drone():
    def __init__(self):
    
        self.state = TelloState()
        self.controler = TelloController()
        self._sensor()

    def _sensor(self):
        _ts_sub = rospy.Subscriber("/tello/status", TelloStatus, self._ts_cb, queue_size = 10)
        _tp_sub = rospy.Subscriber("/target_point", Float64MultiArray, self._tp_cb, queue_size = 10)
        _tg_sub = rospy.Subscriber("/tag_value", Float64MultiArray, self._tg_cb, queue_size = 10)
        
    def _ts_cb(self, data):
        self.state.height = data.height_m
        self.state.temperature_height_m = data.temperature_height_m
        self.state.battery = data.battery_percentage
        self.state.is_flying = data.is_flying
        self.state.fly_mode = data.fly_mode
        
    def _tp_cb(self, data):
        self.state.rec_count = int(data.data[3])
        self.state.target_x[self.state.rec_count] = data.data[0]
        self.state.target_y[self.state.rec_count] = data.data[1]
        self.state.canPass[self.state.rec_count] = data.data[2]

    def _tg_cb(self, data):
        self.state.tag_id = int(data.data[0])

