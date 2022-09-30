#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


#Global variables
angle_order = 0
robot_linear_speed = 0
robot_angular_speed = 0
propotionnal_constant = 0
integration_constant = 0
derivation_constant  = 0
error_topic_name = ""
color_topic_name = ""

robot_command = Twist()

command_publisher = None
subscriberError = None

cumulated_error = 0
previous_error = 0


#Functions to calculate the correction order
def P(measure_angle):
    global angle_order
    rospy.loginfo(measure_angle)
    error = measure_angle - angle_order
    return propotionnal_constant * error

def PI(measure_angle):
    global angle_order
    error = measure_angle - angle_order
    global cumulated_error
    cumulated_error += error
    return P(measure_angle) + integration_constant * cumulated_error

def PID(measure_angle):
    global angle_order
    error = measure_angle - angle_order
    global previous_error
    correction_order = PI(measure_angle) + derivation_constant*(error - previous_error)
    previous_error = error
    return correction_order


#Callback function to calculate the correction order when a message is received on the error topic 
def callbackError(msg):
    robot_command.angular.z = P(msg.data);
    #robot_command.angular.z = PI(msg.data)
    #robot_command.angular.z = PID(msg.data)
    command_publisher.publish(robot_command);


#Function to initialize the node
def RobotPID() :
    global angle_order, robot_linear_speed, robot_angular_speed, propotionnal_constant, integration_constant, derivation_constant, error_topic_name, color_topic_name, robot_command, subscriberError, command_publisher

    angle_order = rospy.get_param("/robotPID/angle_order")

    robot_linear_speed = rospy.get_param("/robotPID/robot_linear_speed")
    robot_angular_speed = rospy.get_param("/robotPID/robot_angular_speed")
    propotionnal_constant = rospy.get_param("/robotPID/propotionnal_constant")
    integration_constant = rospy.get_param("/robotPID/integration_constant")
    derivation_constant = rospy.get_param("/robotPID/derivation_constant")
    error_topic_name = rospy.get_param("/robotPID/error_topic")
    color_topic_name = rospy.get_param("/robotPID/color_topic")

    rospy.init_node('color_sensor_pid_node')

    subscriberError = rospy.Subscriber(error_topic_name, Float32, callbackError)

    command_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1000) 

    robot_command.linear.x = robot_linear_speed
    robot_command.linear.y = 0
    robot_command.linear.z = 0
    robot_command.angular.x = 0 
    robot_command.angular.y = 0 
    robot_command.angular.z = 0 
    
    


RobotPID()
rospy.spin()
