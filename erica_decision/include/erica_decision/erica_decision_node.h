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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//custom header
#include <robotis_math/robotis_math.h>
#include "erica_perception_msgs/PeoplePositionArray.h"




//ros communication

ros::Publisher desired_vector_pub;
ros::Publisher desired_vector_rviz_pub;

ros::Subscriber people_position_sub;



//ros msg
geometry_msgs::Pose desired_vector_msg;
geometry_msgs::PoseStamped desired_vector_rviz_msg;

//variables
Eigen::Quaterniond rqyToQ;



//function
void initialize();
void simulation_rviz(geometry_msgs::Pose desired_vector);

//callback
void people_position_callback(const erica_perception_msgs::PeoplePositionArray::ConstPtr& msg);


#endif /* ERICA_HEROEHS_ERICA_OPERATION_ERICA_DECISION_INCLUDE_ERICA_DECISION_ERICA_DECISION_NODE_H_ */
