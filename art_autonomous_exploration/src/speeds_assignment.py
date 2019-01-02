#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      linear  = 0.
      angular = 0.
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance
      # Constants
      SPEED = 0.3          # Max speed value
      UPPER_LIMIT = 3    # Min value for reducing speed
      LOWER_LIMIT = 2      # Stop distance
      DIRECTIONS = 3       # Number of directions

      # Compute mean distance at every direction
      batch = len(scan) / DIRECTIONS
      distances = [0 for i in range(DIRECTIONS)]
      
      exp = 3       # Bigger range for every direction
      distances[0] = sum(scan[:batch + int(batch/exp)]) / (batch + batch/exp)
      distances[1] = sum(scan[batch - batch/exp:2*batch+ batch/exp])\
                     / (batch + 2*batch/exp)
      distances[2] = sum(scan[2*batch - int(batch/exp):])\
                     / (batch + batch/exp + len(scan)%DIRECTIONS)
      
      # Find max distance and angular direction
      rot_dir = distances.index(max(distances)) - 1

      ang_dir = 1
      if distances[0] > distances[2]:
          ang_dir = -1

      # Add collision distance to the center distance
      t = 2
      if rot_dir:
          distances[1] = (distances[1] + t*min(distances)) / (t+1)

      # Linear equation between linear speed and distance from the obstacle
      # l = a*d + b
      alpha = SPEED/(UPPER_LIMIT-LOWER_LIMIT)
      beta = - LOWER_LIMIT * alpha
      linear = min(alpha*distances[1] + beta, SPEED)
      linear = max(linear, -SPEED)

      # Linear equation between angular speed and distance from obstacle
      alpha_a = SPEED/(LOWER_LIMIT - UPPER_LIMIT)
      beta_a = - UPPER_LIMIT * alpha_a
      angular = max(alpha_a*distances[1] + beta_a, 0)
      angular = min(angular, SPEED)

      angular = angular * ang_dir

      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.
        g_w = 1
        l_w = 0
        if l_goal < 0:
            l_laser = - l_laser
        self.linear_velocity = (g_w*l_goal + l_w*l_laser) / (g_w + l_w)
        self.angular_velocity = (g_w*a_goal + l_w*a_laser) / (g_w + l_w)

        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        self.linear_velocity = l_laser
        self.angular_velocity = a_laser
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
