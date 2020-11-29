#!/usr/bin/env python

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import rospy
import time
import tf
from pyzbar.pyzbar import decode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest

class Planner:
    def __init__(self):
        rospy.init_node('planner')
        self.curr_val = [0,0,0]
        self.box_pos = [0,0,0]
        self.goal_pos = [19.0007046575,71.9998955286,22.1599967919]
        self.scanned_pos = [0,0,0]
        self.dist = [0,0,0,0,0]
        self.min_bottom = 3
        self.min_dist = 2
        self.setpoint = [0,0,0]  
        self.image = 0
        self.image_ = 0
        self.scanned = False
        self.gripped = False
        self.gripWait = 0
        self.adjustWait = 0
        self.init_startpos = False
        self.is_reached = False
        self.destinationUpdated = False
        
        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.laser_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan, self.bottom_callback)
        rospy.Subscriber('/edrone/camera/image_raw',Image,self.image_msg)

        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

        self.newPoint = rospy.Publisher('/setpoint',NavSatFix,queue_size=5)

        self.NewPoint = NavSatFix()

    def inThreshold(self): # checks if drone is in threshold box
        if abs(self.setpoint[0] - self.curr_val[0]) < 0.000004517 and abs(self.setpoint[1] - self.curr_val[1]) < 0.0000047487 and abs(self.setpoint[2] - self.curr_val[2]) < 0.2:
            print(True)
            return True

    def reached(self): # checks if drone is in threshold bo
        if abs(self.goal_pos[0] - self.curr_val[0]) < 0.000004517 and abs(self.goal_pos[1] - self.curr_val[1]) < 0.0000047487:
            print("REACHED")
            return True

    def gps_callback(self,msg):
        self.curr_val = list([msg.latitude,msg.longitude,msg.altitude])

    def laser_callback(self, msg):
        self.dist[:-1] = list(msg.ranges[:-1])

    def bottom_callback(self, msg):
        self.dist[-1] = msg.ranges[0]

    def image_msg(self, msg):
        self.image_ = msg

    def scan_img(self):              # decodes location from qr code

        try:
            if self.image_ != 0:
                bridge = CvBridge()
                self.image = bridge.imgmsg_to_cv2(self.image_,"passthrough")
                data = decode(self.image)
                print(data)
                if data != []:
                    print("DECODED")
                    val = data[0].data
                    print("val",val)
                    self.scanned = True
                    self.scanned_pos = list(map(float,str(val).split(',')))
                else:
                    print("Image not clear")
            else:
                print("image not received")
        except CvBridgeError:
            pass

    def lat_in_range(self):
        if abs(self.curr_val[0] - self.goal_pos[0]) < 0.000004517:
            return True

    def long_in_range(self):
        if abs(self.curr_val[1] - self.goal_pos[1]) < 0.0000047487:
            return True

    def lat_in_threshold(self):
        if abs(self.curr_val[0] - self.setpoint[0]) < 0.000004517:
            return True

    def long_in_threshold(self):
        if abs(self.curr_val[1] - self.setpoint[1]) < 0.0000047487:
            return True

    def alt_in_threshold(self):
        if abs(self.setpoint[2] - self.curr_val[2]) < 0.2:
            return True

    def is_goal_ahead(self): 
        if self.curr_val[1] - self.goal_pos[1] > 0.0000047487:
            return True
        return False

    def is_goal_left(self):
        if self.curr_val[0] - self.goal_pos[0] > 0.000004517:
            return True
        return False

    def moveBack(self):
        self.setpoint[1] = self.curr_val[1]
        self.setpoint[1] += 0.000000903407*5
        print("Moving Backward")
            
    def moveFront(self):
        self.setpoint[1] = self.curr_val[1]
        self.setpoint[1] -= 0.000000903407*5 #long
        print("Moving Forward")

    def moveRight(self):
        self.setpoint[0] = self.curr_val[0]
        self.setpoint[0] += 0.000000949739690705*5 #lat       
        print("Moving Right")

    def moveLeft(self):
        self.setpoint[0] = self.curr_val[0]
        self.setpoint[0] -= 0.000000949739690705*5
        print("Moving Left")


    def keepSafe(self):                    # prevents drone from getting too close to obstacles
        
        if self.dist[0] < self.min_dist and self.dist[0] > 0.5:
            self.moveBack()
            print("FRONT")

        if self.dist[1] < self.min_dist and self.dist[1] > 0.5:
            self.moveLeft()
            print("RIGHT")

        if self.dist[2] < self.min_dist and self.dist[2] > 0.5:
            self.moveFront()
            print("BACK")
        
        if self.dist[3] < self.min_dist and self.dist[3] > 0.5:
            self.moveRight()
            print("LEFT ")
        

        
    def checker(self):        # checks if obstacles are close
        for i in self.dist[:-1]:
            if i < self.min_dist and i > 0.5:  # > 0.5 used to rule out sensor errors
                return True

    
    def avoider(self):         # avoids obstacles by going around them
        obstacleAhead = False
        obstacleSide = False

        if self.is_goal_ahead() and 0.5 < self.dist[0] < 5:
            self.moveLeft()
            obstacleAhead = True

        elif not self.is_goal_ahead() and 0.5 < self.dist[2] < 5:
            self.moveRight()
            obstacleAhead = True

        if not obstacleAhead:
            if self.is_goal_left() and 0.5 < self.dist[3] < 5:
                self.moveBack()
                obstacleSide = True

            elif not self.is_goal_left() and 0.5 < self.dist[1] < 5:
                self.moveFront()
                obstacleSide = True

        return obstacleAhead or obstacleSide
        
    def main(self):
        if not self.init_startpos and self.curr_val != [0,0,0]:    # initializes and gives goal location as box location
            self.setpoint = list(self.curr_val)
            self.setpoint[2] = self.curr_val[2] + 5
            print(self.setpoint)
            self.init_startpos = True

       
        if self.destinationUpdated:
            if abs(self.scanned_pos[0] - self.curr_val[0]) < 0.000004517 and abs(self.scanned_pos[1] - self.curr_val[1]) < 0.0000047487:  # goal location
                self.setpoint[:-1] = list(self.goal_pos[:-1])                                                                         # updates to destination location
                self.adjustWait += 1
                if self.adjustWait > 25:                           # waits for drone to align in x y before descending
                    self.setpoint[2] = self.goal_pos[2]+ 0.05      # 0.05 added because box is going underground and causing problems

        if self.reached() and not self.gripped:
            print("SCANNING")
            if not self.scanned:
                self.scan_img()
            
            self.setpoint[:-1] = list(self.goal_pos[:-1])
            self.adjustWait += 1
            if self.adjustWait > 25:
                self.setpoint[2] = self.goal_pos[2] + 1
                if self.scanned:                                 # waits till qr is decoded then grips the box
                    print("Scan Successful")                      
                    self.setpoint[2] = self.goal_pos[2]
                    resp = self.gripper(True)
                    if resp.result:
                        self.gripped = True
                        self.adjustWait = 0
                        time.sleep(1)
                        self.goal_pos = list(self.scanned_pos)
                        self.destinationUpdated = True 
                        self.setpoint[2] = self.curr_val[2] + 5

        if self.inThreshold() and self.checker():                        
            print("Unsafe")
            self.keepSafe()

        elif self.inThreshold() and self.avoider():
            print("Obstacle avoiding")

        else:
            print("Safe")

            if self.long_in_threshold() and self.alt_in_threshold():
                if not self.long_in_range():
                    if self.is_goal_ahead():
                        self.moveFront()
                    else:
                        self.moveBack()
                else:
                    print("LONG IN RANGE")

            if self.lat_in_threshold() and self.alt_in_threshold():
                if not self.lat_in_range():

                    if self.is_goal_left():
                        self.moveLeft()
                    else:
                        self.moveRight()
                else:
                    print("LAT IN RANGE")



        self.NewPoint.latitude = self.setpoint[0]         # publishes setpoints
        self.NewPoint.longitude = self.setpoint[1]
        self.NewPoint.altitude = self.setpoint[2]

        self.newPoint.publish(self.NewPoint)
        

        

if __name__ == '__main__':

    planner = Planner()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        planner.main()
        r.sleep()


        

