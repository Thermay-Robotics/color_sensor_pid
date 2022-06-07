#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

angle_order = 0
robot_linear_speed = 0
robot_angular_speed = 0
propotionnal_constant = 0
integration_constant = 0
derivation_constant  = 0
sub_topic_name = ""

robot_command = Twist()

command_publisher = None
subscriberError = None


def P(measure_angle):
    error = measure_angle - angle_order
    return propotionnal_constant * error

def callbackError(msg):
    robot_command.angular.z = P(msg.data);
    command_publisher.publish(robot_command);


def RobotPID() : #cumulated_error(0), previous_error(0)
    global angle_order, robot_linear_speed, robot_angular_speed, propotionnal_constant, integration_constant, derivation_constant, sub_topic_name, robot_command, subscriberError, command_publisher

    angle_order = rospy.get_param("/robotPID/angle_order")

    robot_linear_speed = rospy.get_param("/robotPID/robot_linear_speed")
    robot_angular_speed = rospy.get_param("/robotPID/robot_angular_speed")
    propotionnal_constant = rospy.get_param("/robotPID/propotionnal_constant")
    integration_constant = rospy.get_param("/robotPID/integration_constant")
    derivation_constant = rospy.get_param("/robotPID/derivation_constant")
    sub_topic_name = rospy.get_param("/robotPID/sensor_topic")


    rospy.init_node('color_sensor_pid_node')



    #Verifier que le nom du topic est le bon
    subscriberError = rospy.Subscriber(sub_topic_name, Float32, callbackError)
    #Faire pareil avec Color


    command_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1000) 

    robot_command.linear.x = robot_linear_speed
    robot_command.linear.y = 0
    robot_command.linear.z = 0
    robot_command.angular.x = 0 
    robot_command.angular.y = 0 
    robot_command.angular.z = 0 
    
    


RobotPID()
rospy.spin()
"""
double RobotPID::bang_bang(double measure_angle){
    double error = measure_angle - angle_order;
    double calculated_order;
    if(measure_angle - angle_order < 0)
        return -robot_angular_speed;
 
    return robot_angular_speed;
}
"""



#A transformer
"""
double RobotPID::P(double measure_angle){
    double error = measure_angle - angle_order;
    double calculated_order;
    
    return propotionnal_constant * error;

}
"""

"""
double RobotPID::PI(double measure_angle){
    double error = measure_angle - angle_order;

    cumulated_error += error;

    return P(measure_angle) + integration_constant * cumulated_error;
}

double RobotPID::PID(double measure_angle){
    double error = measure_angle - angle_order;

    double correction_order = PI(measure_angle) + derivation_constant*(error - previous_error);

    previous_error = error;

    return correction_order;
}
"""
