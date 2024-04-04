#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import statistics
import time
 

class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver')
        self.pub = rospy.Publisher('/rover/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist = Twist()

        # Parameters
        self.min_distance  = 0.6
        self.wall_distance = 0.5
        

    def control_loop(self):
        if self.min_right_distance < self.min_distance:
            # Too close to the wall, turn left
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.3
            rospy.loginfo("Turning left... Linear_Vel: %f  , Angular_vel: %f", self.twist.linear.x, self.twist.angular.z)

           

        elif self.min_right_distance > self.wall_distance:
            # Too far from the wall, turn right
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.3
            rospy.loginfo("Turning right... Linear_Vel: %f  , Angular_vel: %f", self.twist.linear.x, self.twist.angular.z)
            
            if self.max_front_distance >= 9.0 and self.max_right_distance >= 9.0:
                # Close the ROS node. Destination reached!
                self.twist.linear.x  = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
                time.sleep(2)
                rospy.loginfo("Destination Reached!!")
                rospy.signal_shutdown("Destination Reached!!")
                
            
    
        else:
           
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            rospy.loginfo("Moving forward... Linear_Vel: %f  , Angular_vel: %f", self.twist.linear.x, self.twist.angular.z)

        # Publish Twist message
        self.pub.publish(self.twist)



    def laser_callback(self, scan):
        # Get laser scan data
        ranges = scan.ranges

        # Process laser scan data
        self.min_right_distance = min(ranges[300:359])  # Right side ranges from 300 to 359 degrees
        self.max_right_distance = ranges[290]           # Right side ranges at 290 degrees
        self.max_front_distance = statistics.mean(ranges[0:20] + ranges[358:359] )           # Front side ranges at 359 degrees

        if self.min_right_distance > 9: self.min_right_distance = 9.0
        if self.max_right_distance > 9: self.max_right_distance = 9.0
        if self.max_front_distance > 9: self.max_front_distance = 9.0
        
        # Control logic

        self.control_loop()


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Maze Started...")
    solver = MazeSolver()
    solver.run()
