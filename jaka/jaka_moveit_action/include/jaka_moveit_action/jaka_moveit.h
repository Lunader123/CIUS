#ifndef _JAKA_MOVEIT_H_
#define _JAKA_MOVEIT_H_


#include "libs/robot.h"
#include  <sensor_msgs/JointState.h>


JAKAZuRobot robot;
std::string robot_IP = "192.168.1.200";

bool Robot_Connect_Flag = false;
bool Robot_Move_Flag = false;
bool Robot_State_Thread_Flag = false;

//ros::NodeHandle nh;
//ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("robot_moveit_action", 10);;

//void* Robot_State_Thread(void *threadid);



#endif