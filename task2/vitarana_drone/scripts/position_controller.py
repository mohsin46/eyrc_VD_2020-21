#!/usr/bin/env python



# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import rospy
import time
import tf



def inBound(x):  # keeps required roll and pitch values in range
    if x<1000:
        x = 1000
    elif x > 2000:
        x = 2000
    return x

def inBoundAlt(x): # keeps altitude output in range
    if x<0:
        x = 0
    elif x > 1024:
        x = 1024
    return x

def inThreshold(cmd,val): # checks if drone is in threshold box
    if abs(cmd[0] - val[0]) < 0.000004517 and abs(cmd[1] - val[1]) < 0.0000047487 and abs(cmd[2] - val[2]) < 0.2:
            print(True)
            return True

class Edrone():
    # Position Controller for Edrone
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node 

        self.setpoint = [0,0,0]
        self.Kp = [1.75, 1.75, 70.52]
        self.Ki = [0, 0, 0]
        self.Kd = [45, 45, 1002.3]

        self.prev_values = [0,0,0]
        self.error_values = [0,0,0]
        self.max_values = [2000, 2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000, 1000]
        self.x_error = [0, 0, 0]
        self.y_error = [0, 0, 0]
        self.altitude_error = [0, 0, 0]
        self.req_roll = 0
        self.req_pitch = 0
        self.x_val=0
        self.y_val=0
        self.altitude_val=0
        self.altitude_out=0
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0
        self.pwm_cmd.prop2 = 0
        self.pwm_cmd.prop3 = 0
        self.pwm_cmd.prop4 = 0
        self.image = 0
        self.init_setpos = False

        # Publishing /edrone/pwm, /x_error, /y_error, /altitude_error
        self.x_pub = rospy.Publisher('/req_roll', Float32, queue_size=10)
        self.y_pub = rospy.Publisher('/req_pitch',Float32,queue_size=10)
        self.altitude_pub = rospy.Publisher('/req_altitude',Float32,queue_size=10)
        self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=10)
        self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=10)
        self.altitude_error_pub = rospy.Publisher('/altitude_error', Float32, queue_size=10)
        
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/setpoint', NavSatFix, self.setpoint_init) # takes setpoints published by planner

    
    def gps_callback(self, msg):
        self.x_val = msg.latitude
        self.y_val = msg.longitude
        self.altitude_val = msg.altitude

    def setpoint_init(self, msg):           
        self.setpoint[0] = msg.latitude
        self.setpoint[1] = msg.longitude
        self.setpoint[2] = msg.altitude
        self.init_setpos = True


    def pid(self):

        if inThreshold(self.setpoint, [self.x_val,self.y_val,self.altitude_val]):
            print(True)
                   

        self.x_error[0] = self.setpoint[0]*10**5 - self.x_val*10**5  # error values are very low, hence multiplied to get workable values
        self.y_error[0] = self.setpoint[1]*10**5 - self.y_val*10**5
        self.altitude_error[0] = self.setpoint[2] - self.altitude_val


        if self.x_error[1]*self.Ki[0] < 1024:
            self.x_error[1] += self.x_error[0] 
        
        self.x_error[2] = self.x_error[0] - self.prev_values[0]


        if self.y_error[1]*self.Ki[1] < 1024:
            self.y_error[1] += self.y_error[0] 
        
        self.y_error[2] = self.y_error[0] - self.prev_values[1]


        if self.altitude_error[1]*self.Ki[2] < 1024:
            self.altitude_error[1] += self.altitude_error[0] 
        
        self.altitude_error[2] = self.altitude_error[0] - self.prev_values[2]

        #output for x y altitude

        self.req_roll = 1500 + self.Kp[0]*self.x_error[0] + self.Ki[0]*self.x_error[1] + self.Kd[0]*self.x_error[2]
        self.req_pitch = 1500 + self.Kp[1]*self.y_error[0] + self.Ki[1]*self.y_error[1] + self.Kd[1]*self.y_error[2]
        self.altitude_out = self.Kp[2]*self.altitude_error[0] + self.Ki[2]*self.altitude_error[1] + self.Kd[2]*self.altitude_error[2]

        self.prev_values = self.x_error[0], self.y_error[0], self.altitude_error[0]

        self.x_pub.publish(self.req_roll)
        self.y_pub.publish(self.req_pitch)
        self.altitude_pub.publish(self.altitude_out)
        self.x_error_pub.publish(self.x_error[0]*10)
        self.y_error_pub.publish(self.y_error[0]*10)
        self.altitude_error_pub.publish(self.altitude_error[0])

        print(self.setpoint)

    def main(self):
        if self.init_setpos:    # waits for planner to publish
            self.pid()
        else:
            print("Not Init")

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(10) 
    time.sleep(0.5)             # a small waiting time to allow model to get stable
    while not rospy.is_shutdown():
        e_drone.main()
        r.sleep()

