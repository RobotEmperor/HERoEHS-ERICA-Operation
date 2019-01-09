/*
 * erica_decision_node.h
 *
 *  Created on: Jan 8, 2019
 *      Author: robotemperor
 */

#ifndef ERICA_HEROEHS_ERICA_OPERATION_ERICA_DECISION_INCLUDE_ERICA_DECISION_ERICA_DECISION_NODE_H_
#define ERICA_HEROEHS_ERICA_OPERATION_ERICA_DECISION_INCLUDE_ERICA_DECISION_ERICA_DECISION_NODE_H_

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//custom header
#include <robotis_math/robotis_math.h>


//ros communication



//ros msg


//variables
ros::Time count;


//function
void initialize();


#endif /* ERICA_HEROEHS_ERICA_OPERATION_ERICA_DECISION_INCLUDE_ERICA_DECISION_ERICA_DECISION_NODE_H_ */
