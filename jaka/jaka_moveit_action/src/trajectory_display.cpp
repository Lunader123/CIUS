#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
//#include <sstream>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"

std::fstream File;

void trajectoryCallback(const moveit_msgs::DisplayTrajectory& path)
{
    float path_pos[6];
    float path_vel[6];
    float path_acc[6];
    int point_num;

    //moveit_msgs::RobotTrajectory robot_tra = path.trajectory;
    moveit_msgs::DisplayTrajectory::_trajectory_type _traje = path.trajectory;
    //moveit_msgs::RobotTrajectory::_joint_trajectory_type _joint;
    trajectory_msgs::JointTrajectory::_points_type _point = _traje[0].joint_trajectory.points;

    //std::cout.setf(std::ios::fixed,std::ios::floatfield); 
    //std::cout.precision(3);

    File.open("/home/lzx/Desktop/jakausr/moveit/trajectory.txt", std::ios::app);
    //control_msgs::FollowJointTrajectoryAction action;
    //_joint = _traje[0];
    //_joint.points[1];
    //std::cout << _traje[0] << ' ' << _traje[1];
    //std::cout << _traje[0] << std::endl;
    //_traje[0].joint_trajectory.points
    //std::cout << _point[0] << std::endl;
    point_num = _point.size();
    std::cout << point_num <<std::endl;

    for(int i = 0; i < point_num; i++)
    {
        //std::cout << i << std::endl;
        //std::cout << _point[i] << std::endl;

        for(int j = 0; j < 6; j++)
        {
            path_pos[j] = _point[i].positions[j];
            path_vel[j] = _point[i].velocities[j];
            path_acc[j] = _point[i].accelerations[j];
            std::cout << j+1 << " pos: " << path_pos[j] << " vel: " << path_vel[j] << " acc: " << path_acc[j] << std::endl;            
            File << path_vel[j] << "\t";
        }
        std::cout << std::endl;
        File << std::endl;
        //path_data[i] = path.trajectory.data
    }
    File.close();
    //std::cout << path.trajectory.joint_trajectory;
    
}



int main(int argc, char** argv)
{

    File.open("/home/lzx/Desktop/jakausr/moveit/trajectory.txt", std::ios::out);
    File.clear();
    File.close();

    File.setf(std::ios::fixed, std::ios::floatfield);
    File.precision(4);

    ros::init(argc,argv,"trajectory_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/move_group/display_planned_path",1000,trajectoryCallback);
    ros::spin();

    return 0;
}