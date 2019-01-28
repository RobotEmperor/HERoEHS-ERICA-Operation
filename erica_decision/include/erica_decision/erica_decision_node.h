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
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//custom header
#include <robotis_math/robotis_math.h>
#include "erica_perception_msgs/PeoplePositionArray.h"
#include "robotis_math/fifth_order_polynomial_trajectory.h"
#include "heroehs_math/fifth_order_trajectory_generate.h"


//ros communication
ros::Publisher desired_vector_pub;

// simulation
ros::Publisher desired_vector_rviz_pub;

//gazebo
ros::Publisher left_wheel_front_steering_position_pub;
ros::Publisher right_wheel_front_steering_position_pub;
ros::Publisher left_wheel_rear_steering_position_pub;
ros::Publisher right_wheel_rear_steering_position_pub;

ros::Publisher cmd_vel_pub;

//ros msg
geometry_msgs::Pose desired_vector_msg;
geometry_msgs::PoseStamped desired_vector_rviz_msg;

std_msgs::Float64 left_wheel_front_steering_position_msg;
std_msgs::Float64 right_wheel_front_steering_position_msg;
std_msgs::Float64 left_wheel_rear_steering_position_msg;
std_msgs::Float64 right_wheel_rear_steering_position_msg;

geometry_msgs::Twist cmd_vel_msg;

//variables
Eigen::Quaterniond rqyToQ;
ros::Time count;

bool   simulation_check;
double simulation_robot_speed;

double robot_trj_time;
double goal_desired_vector_x;
double goal_desired_vector_y;

//math class
heroehs_math::FifthOrderTrajectory * fifth_trj_x;
heroehs_math::FifthOrderTrajectory * fifth_trj_y;

//function
void initialize();
void simulation_rviz(geometry_msgs::Pose desired_vector);
void simulation_gazebo(geometry_msgs::Pose desired_vector);

//function algorithm
//void desired_vector_trj(geometry_msgs::Pose *out_desired_vector_, double desired_value_x_ , double desired_value_y_);

//callback
void people_position_callback(const erica_perception_msgs::PeoplePositionArray::ConstPtr& msg);
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);


#endif /* ERICA_HEROEHS_ERICA_OPERATION_ERICA_DECISION_INCLUDE_ERICA_DECISION_ERICA_DECISION_NODE_H_ */
