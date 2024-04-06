#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi
import statistics
import time

class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver')
        self.cmd_vel_pub = rospy.Publisher('/rover/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist = Twist()

        # Parameters
        self.min_distance  = 0.6  # Minimum distance to maintain from the wall
        self.wall_distance = 0.5  # Desired distance from the wall
       
        self.actualError   = 0.0 
        self.newError = 0.0 
        self.preError = 0.0
        self.kp = 10
        self.kd = 3
        self.ki = 0.0

    def control_loop(self):
        if self.min_right_distance < self.min_distance:
            # Too close to the wall, turn left
            self.twist.linear.x = 0.1
            self.twist.angular.z = self.angularResponse
            rospy.loginfo("Turning left... Linear_Vel: %f  , Angular_vel: %f", self.twist.linear.x, self.twist.angular.z)

            

        elif self.min_right_distance > self.wall_distance:
            # Too far from the wall, turn right
            self.twist.linear.x = 1.0
            self.twist.angular.z = self.angularResponse
            rospy.loginfo("Turning right... Linear_Vel: %f  , Angular_vel: %f", self.twist.linear.x, self.twist.angular.z)
            
            if self.max_front_distance >= 9.0 and self.max_right_distance >= 9.0:
                # Close the ROS node. Destination reached!
                self.twist.linear.x  = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(2)
                rospy.loginfo("Destination Reached")
                rospy.signal_shutdown("Destination Reached")
                
           
       
        else:
            # Maintain desired distance from the wall
            self.twist.linear.x = 2.0
            self.twist.angular.z = 0
            rospy.loginfo("Moving forward... Linear_Vel: %f  , Angular_vel: %f", self.twist.linear.x, self.twist.angular.z)

        
        self.cmd_vel_pub.publish(self.twist)

    def PID(self):
        self.actualError = (self.min_distance - self.min_right_distance)

        errorPropotional = self.actualError * self.kp 

        errorIntegral = (self.newError + self.actualError) * self.ki
        self.newError = errorIntegral

        errorDerivative =  self.kd * (self.actualError - self.preError)
        self.preError = self.actualError

        self.angularResponse =  (errorPropotional + errorIntegral + errorDerivative)

        if(self.angularResponse > pi/2):
            self.angularResponse = pi/2
        elif(self.angularResponse < -1 * pi/2):
            self.angularResponse = -1*pi/2

      
            
        #Based on front distance
        if(self.max_front_distance < 3):
            self.leanearResponse = 2.0 + (abs(3-self.max_front_distance)) 



    def laser_callback(self, scan):
        # Get laser scan data
        ranges = scan.ranges

        # Process laser scan data
        self.min_right_distance = min(ranges[300:359])  # Right side ranges from 300 to 359 degrees
        self.max_right_distance = ranges[290]           # Right side ranges at 290 degrees
        self.max_front_distance = statistics.mean(ranges[0:20] + ranges[358:359] )          

        if self.min_right_distance > 9: self.min_right_distance = 9.0
        if self.max_right_distance > 9: self.max_right_distance = 9.0
        if self.max_front_distance > 9: self.max_front_distance = 9.0
        
        # Control logic
        self.PID()
        self.control_loop()


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Maze Started...")
    solver = MazeSolver()
    solver.run()