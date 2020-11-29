#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


def inBound(x):   # to keep pwm values in range
    if x<0:
        x = 0
    elif x > 1024:
        x = 1024
    return x

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500, 1500, 1500]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0
        self.pwm_cmd.prop2 = 0
        self.pwm_cmd.prop3 = 0
        self.pwm_cmd.prop4 = 0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [20, 20, 250]
        self.Ki = [0, 0, 0]
        self.Kd = [650, 650, 0]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
        self.prev_values = [0,0,0]
        self.error_values = [0,0,0]
        self.roll_error = [0, 0, 0]
        self.pitch_error = [0, 0, 0]
        self.yaw_error = [0, 0, 0]
        self.out_roll = 0
        self.out_pitch = 0
        self.out_yaw = 0
        self.out_altitude=0
        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=10)
        self.roll_error_pub = rospy.Publisher('/roll_error',Float32, queue_size=10)
        self.yaw_error_pub = rospy.Publisher('/yaw_error',Float32, queue_size=10)
        self.pitch_error_pub = rospy.Publisher('/pitch_error',Float32,queue_size=10)

        

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw

        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/req_roll', Float32, self.roll_init)
        rospy.Subscriber('/req_pitch', Float32, self.pitch_init)
        rospy.Subscriber('/req_altitude', Float32, self.altitude_control)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)


        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

    def roll_init(self, msg):
        self.setpoint_cmd[0] = msg.data

    def pitch_init(self, msg):
        self.setpoint_cmd[1] = msg.data


    def altitude_control(self, msg):
        self.out_altitude = msg.data
        # ---------------------------------------------------------------------------------------------------------------
    
    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.05  
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.1
        
    def pitch_set_pid(self, pitch):
        self.Kp[2] = pitch.Kp * 0.05  
        self.Ki[2] = pitch.Ki * 0.008
        self.Kd[2] = pitch.Kd * 0.1


    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    #TODO
    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------
        #self.setpoint_cmd=[1500,1500,1500]
        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles
        #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
        #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   9. Add error_sum to use for integral component

        # Converting quaternion to euler angles
        #1
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        # Complete the equations for pitch and yaw axis
        #2
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        #print("euler",self.setpoint_cmd[1], self.setpoint_euler[1])


        # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itself
        
        #3
        self.error_values[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0] 
        self.error_values[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1] 
        self.error_values[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]
        #print("error",self.error_values[1],"set",self.setpoint_euler[1],"curr",self.drone_orientation_euler[1])

        #4
        #P I D errors for roll 
        self.roll_error[0] = self.error_values[0]

        if self.roll_error[1]*self.Ki[0] < 1024:
            self.roll_error[1] += self.error_values[0] 
        
        self.roll_error[2] = self.error_values[0] - self.prev_values[0]

        #P I D errors for pitch
        self.pitch_error[0] = self.error_values[1]

        if self.pitch_error[1]*self.Ki[1] < 1024:
            self.pitch_error[1] += self.error_values[1] 
        
        self.pitch_error[2] = self.error_values[1] - self.prev_values[1]

        #P I D errors for yaw
        self.yaw_error[0] = self.error_values[2]

        if self.yaw_error[1]*self.Ki[2] < 1024:
            self.yaw_error[1] += self.error_values[2] 
        
        self.yaw_error[2] = self.error_values[2] - self.prev_values[2]
        

        #output for roll pitch yaw
        
        self.out_roll = self.Kp[0]*self.roll_error[0] + self.Ki[0]*self.roll_error[1] + self.Kd[0]*self.roll_error[2]
        self.out_pitch = self.Kp[1]*self.pitch_error[0] + self.Ki[1]*self.pitch_error[1] + self.Kd[1]*self.pitch_error[2]
        self.out_yaw = self.Kp[2]*self.yaw_error[0] + self.Ki[2]*self.yaw_error[1] + self.Kd[2]*self.yaw_error[2]

        #converts this output to throttle command 

        self.pwm_cmd.prop1 =   510 + self.out_altitude - self.out_roll + self.out_pitch - self.out_yaw
        self.pwm_cmd.prop2 =   510 + self.out_altitude - self.out_roll - self.out_pitch + self.out_yaw
        self.pwm_cmd.prop3 =   510 + self.out_altitude + self.out_roll - self.out_pitch - self.out_yaw
        self.pwm_cmd.prop4 =   510 + self.out_altitude + self.out_roll + self.out_pitch + self.out_yaw

        
        self.pwm_cmd.prop1 = inBound(self.pwm_cmd.prop1)
        self.pwm_cmd.prop2 = inBound(self.pwm_cmd.prop2)
        self.pwm_cmd.prop3 = inBound(self.pwm_cmd.prop3)
        self.pwm_cmd.prop4 = inBound(self.pwm_cmd.prop4)        
        #
        #
        print("prop1",self.out_altitude , self.out_roll , self.out_pitch , self.out_yaw)
        print("prop2",self.out_altitude , self.out_roll , self.out_pitch , self.out_yaw)
        print("prop3",self.out_altitude , self.out_roll , self.out_pitch , self.out_yaw)
        print("prop4",self.out_altitude , self.out_roll , self.out_pitch , self.out_yaw)
        self.prev_values = list(self.error_values)
        # ------------------------------------------------------------------------------------------------------------------------

        self.pwm_pub.publish(self.pwm_cmd)
    
        self.roll_error_pub.publish(self.roll_error[0])
        self.pitch_error_pub.publish(self.pitch_error[0])
        self.yaw_error_pub.publish(self.yaw_error[0])

        #print(roll_error)
        #print("roll",self.Kp[0]*self.roll_error[0] , self.Ki[0]*self.roll_error[1] , self.Kd[0]*self.roll_error[2])
        #print(pitch_error)
        #print("pitch",self.Kp[1]*self.pitch_error[0] , self.Ki[1]*self.pitch_error[1] , self.Kd[1]*self.pitch_error[2])
        print("yaw",self.Kp[2]*self.yaw_error[0] , self.Ki[2]*self.yaw_error[1] , self.Kd[2]*self.yaw_error[2])

        print("errors", self.setpoint_euler[2], self.drone_orientation_euler[2], self.yaw_error[0])

        print("yaw_out",self.out_yaw)

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(10)
    time.sleep(0.5)  
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()

