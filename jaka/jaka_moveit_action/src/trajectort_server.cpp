#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <thread>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <actionlib/server/simple_action_server.h>
#include "jaka_moveit_action/jakacontrollerAction.h"

#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
//#include  <sensor_msgs/JointState.h>
#include  <std_msgs/Empty.h>

#include "jaka_moveit_action/jakacontrollerAction.h"

#include "jaka_moveit.h"
//#include "../include/jaka_moveit.h"

//action的定义
typedef actionlib::SimpleActionServer<jaka_moveit_action::jakacontrollerAction> Server;

//执行任务发布用的发布器定义
ros::Publisher current_pose;
ros::Publisher action_pub;

void jakacontroller_execute(const jaka_moveit_action::jakacontrollerGoalConstPtr& goal, Server *ac)
{
    //ros::Rate(1);
    //ros::Rate r(1);

    int point_num = 0;//point executed
    int total_point_num = goal->point_count;

    jaka_moveit_action::jakacontrollerFeedback feedback;
    //sensor_msgs::JointState joint_states;

    JointValue joint_goal, joint_now;
    CartesianPose cart_goal;
    bool Robot_Position_Flag = false;

    float val = 0;

    //read the series of points
    std::vector<float> Joint_Series;
    //std::cout << total_point_num << std::endl;
    //std::cout << goal->robot_goal[0] << ' ' << goal->robot_goal[1] << ' ' << goal->robot_goal[2]<< std::endl;
    //Joint_Series.clear();
    for(int i = 0; i < total_point_num * 6; i++)
    {
        val = goal->robot_goal[i];
        Joint_Series.push_back(val);
        //std::cout << Joint_Series[i] << std::endl;
    }
    
    /*joint_goal.jVal[0] = Joint_Series[0];
    joint_goal.jVal[1] = Joint_Series[1];
    joint_goal.jVal[2] = Joint_Series[2];
    joint_goal.jVal[3] = Joint_Series[3];
    joint_goal.jVal[4] = Joint_Series[4];
    joint_goal.jVal[5] = Joint_Series[5];*/

    

    /* for(int i = 0; i < 6; i++)
    {
        joint_goal.jVal[i] = goal->robot_goal[i];
    }
    std::cout << "Joint goal: " << std::endl; */
    
    if(Robot_Connect_Flag && !Robot_Move_Flag)
    {
        robot.servo_move_use_joint_LPF(4);
        robot.servo_move_enable(true);

        std::cout << "Real robot trajectory set" << std::endl;

        for(point_num = 0; point_num < total_point_num; point_num++)
        {
            joint_goal.jVal[0] = Joint_Series[0 + point_num * 6];
            joint_goal.jVal[1] = Joint_Series[1 + point_num * 6];
            joint_goal.jVal[2] = Joint_Series[2 + point_num * 6];
            joint_goal.jVal[3] = Joint_Series[3 + point_num * 6];
            joint_goal.jVal[4] = Joint_Series[4 + point_num * 6];
            joint_goal.jVal[5] = Joint_Series[5 + point_num * 6];

            robot.servo_j(&joint_goal, ABS);
            usleep(16 * 1000);

            
            //robot.get_joint_position(&joint_now);
            //MoveMode s;

            /* feedback.robot_now.clear();
            for(int i = 0; i < 6; i ++)
            {
                feedback.robot_now.push_back(joint_now.jVal[i]);
            }

            feedback.point_num = point_num + 1; */

            /*joint_states.position.clear();
            for(int i = 0; i < 6; i++)
            {
                joint_states.position.push_back(joint_goal.jVal[i]);
                int j = i+1;
                joint_states.name.push_back("joint_"+ std::to_string(j));
                joint_states.header.stamp = ros::Time::now();
            }

            action_pub.publish(joint_states);*/
            
        }

        std::cout << "Real robot move enable" << std::endl;

        sensor_msgs::JointState joint_states;

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

        //while()
            //Robot_Position_Flag = false;

            //robot.kine_forward(&joint_goal, &cart_goal);
            //std::cout << cart_goal.tran.x << cart_goal.tran.y << std::endl;

            //if(!robot.joint_move(&joint_goal, ABS, false, 0.5, 0.5, 0.1, NULL))
            //if(!robot.linear_move(&cart_goal, ABS, false, 5, 0.5, 0.1, NULL))
            //{
                /*while(!Robot_Position_Flag)
                {
                    

                    for(int i = 0; i < 6; i ++)
                    {
                        if(fabs(joint_goal.jVal[i] - joint_now.jVal[i]) > 0.01)
                        {
                            Robot_Position_Flag = false;
                            break;
                        }
                        else
                        {
                            Robot_Position_Flag = true;
                        }
                    }
                }*/

                feedback.robot_now.clear();
                for(int i = 0; i < 6; i ++)
                {
                    feedback.robot_now.push_back(joint_now.jVal[i]);
                }

                feedback.point_num = point_num + 1;                

                ac->publishFeedback(feedback);

            robot.servo_move_enable(false);
            //}
            //else
            //{
                //std::cout << "Error in robot joint move!" << std::endl;
            //}
            /* for(int i = 0; i < 6; i ++)
            {
                if(!fabs(joint_goal.jVal[i] - joint_now.jVal[i]) < 0.1)
                {
                    break;
                }
                Robot_Position_Flag = true;
                //feedback.robot_now[i] = joint_now.jVal[i];
            } */
            //publish robot joint            
                
    }
    else if(!Robot_Move_Flag)
    {
        
        for(point_num = 0; point_num < total_point_num; point_num++)
        {
            joint_goal.jVal[0] = Joint_Series[0 + point_num * 6];
            joint_goal.jVal[1] = Joint_Series[1 + point_num * 6];
            joint_goal.jVal[2] = Joint_Series[2 + point_num * 6];
            joint_goal.jVal[3] = Joint_Series[3 + point_num * 6];
            joint_goal.jVal[4] = Joint_Series[4 + point_num * 6];
            joint_goal.jVal[5] = Joint_Series[5 + point_num * 6];

            sensor_msgs::JointState joint_states;

            joint_states.position.clear();
            for(int i = 0; i < 6; i++)
            {
                joint_states.position.push_back(joint_goal.jVal[i]);
                int j = i+1;
                joint_states.name.push_back("joint_"+ std::to_string(j));
                joint_states.header.stamp = ros::Time::now();
            }

            
            //MoveMode s;
            feedback.robot_now.clear();

            for(int i = 0; i < 6; i ++)
            {
                feedback.robot_now.push_back(joint_goal.jVal[i]);
            }
//std::cout << "1" << std::endl;
            feedback.point_num = point_num + 1;

            /* for(int i = 0; i < 6; i++)
            {
                joint_states.position.push_back(joint_now.jVal[i]);
                int j = i+1;
                joint_states.name.push_back("joint_"+ std::to_string(j));
                joint_states.header.stamp = ros::Time::now();
            } */

            action_pub.publish(joint_states);

            //std::cout << "Publish feedback!" << std::endl;
            //std::cout << feedback.robot_now[1] << std::endl;

            ac->publishFeedback(feedback);

            //r.sleep();

            usleep(20 * 1000);// unit: us
            //sleep(1);          
                //std::cout << "2" << std::endl;
        }
            /* for(int i = 0; i < 6; i ++)
            {
                if(!fabs(joint_goal.jVal[i] - joint_now.jVal[i]) < 0.1)
                {
                    break;
                }
                Robot_Position_Flag = true;
                //feedback.robot_now[i] = joint_now.jVal[i];
            } */
            //publish robot joint

    }
    else
    {
        std::cout << "Waiting for moveit command!" << std::endl;
    }


    ac->setSucceeded();

}

void RVIZ_Joint_Init(JointValue& joint)
{
    sensor_msgs::JointState joint_states;

    joint_states.position.clear();
    for(int i = 0; i < 6; i++)
    {
        joint_states.position.push_back(joint.jVal[i]);
        int j = i+1;
        joint_states.name.push_back("joint_"+ std::to_string(j));
        joint_states.header.stamp = ros::Time::now();
    }

    action_pub.publish(joint_states);
}

void* Robot_State_Thread(void *threadid)
{    
    JointValue joint_now;
    std::cout << "Robot State Thread Start!" << std::endl;

    while(Robot_State_Thread_Flag)
    {
        sensor_msgs::JointState joint_states;
        RobotStatus status;

        //robot.get_joint_position(&joint_now);
        robot.get_robot_status(&status);
        //status.joint_position

        joint_states.position.clear();
        for(int i = 0; i < 6; i++)
        {
            joint_states.position.push_back(status.joint_position[i]);
            int j = i+1;
            joint_states.name.push_back("joint_"+ std::to_string(j));
            joint_states.header.stamp = ros::Time::now();
        }
        //std::cout << "Publish Joints" << std::endl;

        action_pub.publish(joint_states);
        usleep(4 * 1000);
    }

    std::cout << "Robot State Thread End!" << std::endl;

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"trajectory_action_server");
    ros::NodeHandle n;

    JointValue joint_init;

    current_pose = n.advertise<std_msgs::Empty>("/rviz/moveit/update_start_state", 10);

    action_pub = n.advertise<sensor_msgs::JointState>("robot_moveit_action", 10);

    std_msgs::Empty empty;

    //connect
    if(argv[1] != NULL)
    {
        std::cout << "Connect robot IP: " << argv[1] << std::endl;
        robot_IP = argv[1];


        if(!robot.login_in(robot_IP.c_str()))
        {
            if(!robot.power_on())
            {
                if(!robot.enable_robot())
                {
                    pthread_t robot_pose;
                    Robot_State_Thread_Flag = true;

                    std::cout << "Robot connect!" << std::endl;
                    Robot_Connect_Flag = true;

                    robot.get_joint_position(&joint_init);
                    usleep(100 * 1000);

                    RVIZ_Joint_Init(joint_init);
                    pthread_create(&robot_pose, NULL, Robot_State_Thread, NULL);
                }
            }
        }
        else
        {
            std::cout << "Cannot connect robot!" << std::endl;
            exit(-1);
        }
    }
    else
    {
        std::cout << "Use simulated robot!" << std::endl;
        Robot_Connect_Flag = false;

        //pthread_t robot_pose;
        //pthread_create(&robot_pose, NULL, Robot_State_Thread, NULL);
        //return -1;
    }

    usleep(100 * 1000);
    current_pose.publish(empty);
    //ros::Subscriber sub = n.subscribe("/move_group/display_planned_path",1000,trajectoryCallback);

    //Server
    Server server(n, "jaka_moveit_action", boost::bind(&jakacontroller_execute, _1, &server), false);

    std::cout << "Server start" << std::endl;
    server.start();

    //Robot_State_Thread_Flag = false;

    ros::spin();

    return 0;
}
