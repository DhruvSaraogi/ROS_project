#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from math import pi
from enum import Enum
import statistics
import time
import numpy as np


class states(Enum):
    READY = 0
    GOING = 1
    TURNING = 2


class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver')
        self.cmd_vel_pub = rospy.Publisher('/rover/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist = Twist()

        # Parameters
        self.empty_thres  = 0.9  # Minimum distance to maintain from the wall
        self.wall_distance = 0.6  # Desired distance from the wall
        self.crash_warn = 0.36
        self.linear_speed  = .12  # Base Linear speed
        # self.newError = 0.0 
        # self.preError = 0.0
        self.max_ang_vel = 0.5
        
        # variables
        self.listener = tf.TransformListener()
        self.orient = 0 # -1 for right, 1 for left
        self.base_orient = 1 # -1 for right, 1 for left
        self.yaw = 0
        self.aimed_yaw = 0
        self.state = states.GOING
        self.left_crash = False
        self.right_crash = False

        self.kp = 0.5

        self.ki = 0.7

    def control_loop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        crash_move = False

        print("state:",self.state)
        if self.state == states.GOING:   # go forward
            self.twist.linear.x = self.linear_speed
            if(self.left_crash or self.right_crash):    # if rover is very close to one of the sides
                crash_move = True
                self.twist.angular.z = -0.2 if self.left_crash else 0.2
        if self.state == states.TURNING or self.state == states.GOING:  
            try:
                (trans,rot) = self.listener.lookupTransform('/map', '/base_body', rospy.Time(0))    # orientation of the rover obtained from gmapping
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
                print("yaw: ", yaw)
                ang_vel = self.PID(yaw)
                if(abs(ang_vel) > self.max_ang_vel):    # calculate angular velocity and limit it
                    ang_vel = self.max_ang_vel * ang_vel / abs(ang_vel)
                if not crash_move:
                    self.twist.angular.z = ang_vel
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
        # Publish Twist message
        self.cmd_vel_pub.publish(self.twist)

    def PID(self, yaw):
        actualError = (self.aimed_yaw - yaw)    # simple proportional controller
            
        if(actualError > pi):
            actualError -= 2*pi
        elif(actualError < -1*pi):
            actualError += 2*pi

        if(abs(actualError) < 0.1):
            self.yaw = self.aimed_yaw
            self.state = states.GOING

        errorPropotional = actualError * self.kp
        return errorPropotional



    def laser_callback(self, scan):
        # Get laser scan data
        ranges = scan.ranges
        ranges = np.array(ranges)
        ranges[ranges == np.inf] = 10

        # Process laser scan data
        left = min(ranges[70:110])
        front = min(min(ranges[:10]), min(ranges[-10:]))
        right = min(ranges[250:290])
        self.left_empty = left > self.empty_thres       # check sides
        self.front_empty = front > self.wall_distance
        self.right_empty = right > self.empty_thres

        self.left_crash = left < self.crash_warn
        self.right_crash = right < self.crash_warn

        
        if self.state == states.GOING:     # robot on a track and moving forward
            print("left: ",left)
            print("front: ",front)
            print("right: ",right)
            print("orient:",self.orient)
            if not self.front_empty:
                if(self.left_empty and self.base_orient == 1):
                    self.Turn(pi/2)  # turn 90 degree
                    self.orient = -1
                if(self.right_empty  and self.base_orient == -1):
                    self.Turn(-1*pi/2)     # turn 90 degree
                    self.orient = 1
                self.state = states.TURNING
            else:
                (trans,rot) = self.listener.lookupTransform('/map', '/base_body', rospy.Time(0))    
                print("pos:")
                print(trans)
                if(abs(trans[0]+2.5) < 0.1 ):
                    if ((self.base_orient == 1 and self.yaw < -0.5) or(self.base_orient == -1 and self.yaw > 0.5)):
                        self.Turn(-1*self.yaw)
                        self.base_orient *= -1
                        self.state = states.TURNING
                        return
                if(self.base_orient == 1 and self.orient == -1 and not self.right_empty ):
                    self.orient = -2

                if(self.base_orient == -1 and self.orient == 1 and not self.left_empty ):
                    self.orient = 2
                
                if self.base_orient == 1 and self.orient == -2 and self.right_empty:
                    self.Turn(-1*pi/2)  # turn 90 degree
                    self.orient = -1
                    self.state = states.TURNING

                elif self.base_orient == -1 and self.orient == 2 and self.left_empty:
                    self.orient = 1
                    self.Turn(pi/2)     # turn 90 degree
                    self.state = states.TURNING

                

        self.min_right_distance = min(ranges[300:359])  # Right side ranges from 300 to 359 degrees
        self.min_left_distance = min(ranges[300:359])  # Right side ranges from 300 to 359 degrees
        self.max_right_distance = ranges[290]           # Right side ranges at 290 degrees
        self.max_front_distance = statistics.mean(ranges[0:20] + ranges[358:359] )           # Front side ranges at 359 degrees

        if self.min_right_distance > 9: self.min_right_distance = 9.0
        if self.max_right_distance > 9: self.max_right_distance = 9.0
        if self.max_front_distance > 9: self.max_front_distance = 9.0
        
        # Control logic
        self.control_loop()

    def Turn(self, yaw_change):     # calc the goal angle to turn
        self.aimed_yaw = self.yaw + yaw_change
        if(self.aimed_yaw > pi):
            self.aimed_yaw -= 2*pi
        elif(self.aimed_yaw < -1*pi):
            self.aimed_yaw += 2*pi
        print("Aimed yaw:",self.aimed_yaw)    


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Maze Started...")
    solver = MazeSolver()
    solver.run()