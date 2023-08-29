#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <actionlib/client/simple_action_client.h>
#include "jaka_moveit_action/jakacontrollerAction.h"

#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
//#include  <sensor_msgs/JointState.h>
#include  <std_msgs/Empty.h>


#include "jaka_moveit_action/jakacontrollerAction.h"

#include "jaka_moveit.h"
 
typedef actionlib::SimpleActionClient<jaka_moveit_action::jakacontrollerAction> Client;

jaka_moveit_action::jakacontrollerGoal goal;

bool Trajectory_Series_Flag = false;
bool Trajectory_Series_End = false;

// 这个是用来在结束的时候刷新的
ros::Publisher start_state_pub;
 
// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const jaka_moveit_action::jakacontrollerResultConstPtr& result)
{
    std_msgs::Empty start_state;

    ROS_INFO("Moveit trajectory is done!");
    start_state_pub.publish(start_state);

    //Robot_State_Thread_Flag = false;

    ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Moveit goal is active!");
}

// 收到feedback后调用的回调函数
void feedbackCb(const jaka_moveit_action::jakacontrollerFeedbackConstPtr& feedback)
{
    //ROS_INFO("Robot joint state:  ");
    std::cout << "Series point num: " << feedback->point_num << std::endl;
    std::cout << "Joint 1: " << feedback->robot_now[0] << std::endl;
    std::cout << "Joint 2: " << feedback->robot_now[1] << std::endl;
    std::cout << "Joint 3: " << feedback->robot_now[2] << std::endl;
    std::cout << "Joint 4: " << feedback->robot_now[3] << std::endl;
    std::cout << "Joint 5: " << feedback->robot_now[4] << std::endl;
    std::cout << "Joint 6: " << feedback->robot_now[5] << std::endl;

    //ros::spinOnce();
}

void trajectoryCallback(const moveit_msgs::DisplayTrajectory& path)
{
    float path_pos[6];
    float path_vel[6];
    float path_acc[6];
    int16_t point_num;

    std::cout << "Recieve trajectory!" << std::endl;

    //moveit_msgs::RobotTrajectory robot_tra = path.trajectory;
    moveit_msgs::DisplayTrajectory::_trajectory_type _traje = path.trajectory;
    //moveit_msgs::RobotTrajectory::_joint_trajectory_type _joint;
    trajectory_msgs::JointTrajectory::_points_type _point = _traje[0].joint_trajectory.points;

    //control_msgs::FollowJointTrajectoryAction action;
    //_joint = _traje[0];
    //_joint.points[1];
    //std::cout << _traje[0] << ' ' << _traje[1];
    //std::cout << _traje[0] << std::endl;
    //_traje[0].joint_trajectory.points
    //std::cout << _point[0] << std::endl;
    point_num = _point.size();
    std::cout << point_num <<std::endl;

    goal.robot_goal.clear();

    for(int i = 0; i < point_num; i++)
    {
        //std::cout << i << std::endl;
        //std::cout << _point[i] << std::endl;

        for(int j = 0; j < 6; j++)
        {
            path_pos[j] = _point[i].positions[j];
            path_vel[j] = _point[i].velocities[j];
            path_acc[j] = _point[i].accelerations[j];
            //std::cout << j+1 << " pos: " << path_pos[j] << " vel: " << path_vel[j] << " acc: " << path_acc[j] << std::endl;

            // push_back函数的作用是在Vector(容器)的最后添加元素
            goal.robot_goal.push_back(path_pos[j]);
        }
        
        //std::cout << std::endl;
        //path_data[i] = path.trajectory.data
    }
    goal.point_count = point_num;
    //std::cout << path.trajectory.joint_trajectory;
    Trajectory_Series_Flag = true;

    //ros::spin();
    
    
}

/*void* Robot_State_Thread(void *threadid)
{
    sensor_msgs::JointState joint_states;
    JointValue joint_now;
    std::cout << "Robot State Thread Start!" << std::endl;

    while(Robot_State_Thread_Flag)
    {
        robot.get_joint_position(&joint_now);

        joint_states.position.clear();
        for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(joint_now.jVal[i]);
            int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
            joint_states.header.stamp = ros::Time::now();
        }

        action_pub.publish(joint_states);
    }

    std::cout << "Robot State Thread End!" << std::endl;

    return 0;
}*/
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_action_client");
    ros::NodeHandle n;
 
    // 定义一个客户端
    Client client("jaka_moveit_action", true);
 
    

    ros::Subscriber trajectory_sub = n.subscribe("/move_group/display_planned_path",1000,trajectoryCallback);

    start_state_pub = n.advertise<std_msgs::Empty>("/rviz/moveit/update_start_state", 10);
    //ros::Subscriber trajectory_sub;

    /*goal.point_count = 1;
    goal.robot_goal.push_back(0);
    goal.robot_goal.push_back(1.57);
    goal.robot_goal.push_back(0);
    goal.robot_goal.push_back(1.57);
    goal.robot_goal.push_back(0);
    goal.robot_goal.push_back(0);*/

    ros::WallTime startr = ros::WallTime::now();
    long time = 0;
    int time_limit = 10000;//timespan limit (unit: ms)

    //waiting for trajectory
    while(1)
    {
        time = (ros::WallTime::now() - startr).toSec()*1000;
        if(time > time_limit)
        {
            std::cout << "Cannot recieve trajectory, please check!" << std::endl;
            return -1;
        }

        if(Trajectory_Series_Flag)
        {
            break;
        }

        //trajectory_sub = n.subscribe("/move_group/display_planned_path",1000,trajectoryCallback);
        //std::cout << Trajectory_Series_Flag << std::endl;
        ros::spinOnce();
        
    }


 
    // 创建一个action的goal
 
    
    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    //std::cout << goal << std::endl;
    /*std::cout << Robot_Connect_Flag << std::endl;
    if(Robot_Connect_Flag)
    {
        pthread_t robot_pose;
        Robot_State_Thread_Flag = true;
        pthread_create(&robot_pose, NULL, Robot_State_Thread, NULL);
    }*/

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);
    //ros::spinOnce();

 
    ros::spin();
 
    return 0;
}