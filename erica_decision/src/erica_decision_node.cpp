/*
 * erica_decision_node.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: robotemperor
 */

#include <erica_decision/erica_decision_node.h>

void initialize()
{

}
//callback
/*void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
}*/

//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mobile_manager_node");


  ros::NodeHandle nh;
  initialize();

  //motor1_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_1",10);

  //joy_sub   = nh.subscribe("/joy", 1, joy_callback);

  while(ros::ok())
  {
   // motor1_pub.publish(motor_cmd_msg_1);

    usleep(100);

    /*printf("---------------------------------------\n");
    printf("DIR Motor1 :: %d \n", motor_cmd_msg_1.motor_desired_direction);
     */
    ros::spinOnce();
  }
  return 0;
}
