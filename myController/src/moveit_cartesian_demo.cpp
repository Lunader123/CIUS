/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ctime>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveit_cartesian_demo");
	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("jaka");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    
    std::cout<<arm.getPlanningFrame()<<std::endl;

    // 将桌面加入障碍物
    clock_t startTime,endTime;
    startTime = clock();
    moveit_msgs::CollisionObject collosion_object;
    collosion_object.header.frame_id = arm.getPlanningFrame();
    moveit::planning_interface::PlanningSceneInterface planning_scene; //环境创建
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.75;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -2.6;
    box_pose.position.y = 1.8;
    box_pose.position.z = 1.2;

    collosion_object.primitives.push_back(primitive);
    collosion_object.primitive_poses.push_back(box_pose);
    collosion_object.operation = collosion_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collosion_objects;
    collosion_objects.clear();
    collosion_objects.push_back(collosion_object);
    planning_scene.addCollisionObjects(collosion_objects);

    endTime = clock();

    std::cout << "The run time is:"<<double(endTime - startTime)/ CLOCKS_PER_SEC << "s" << std::endl;

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(false);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.001);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.7);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 获取当前位姿数据最为机械臂运动的起始位姿
    geometry_msgs::PoseStamped start_pose = arm.getCurrentPose(end_effector_link);
    std_msgs::Header header_frame;
    header_frame.frame_id = "base_link";
    start_pose.header = header_frame;

	std::vector<geometry_msgs::Pose> waypoints;

    std::cout<<"Start_Pose_Orientation = \n"<<start_pose.header<<std::endl;

    std::cout<<"Start_Pose_Orientation = \n"<<start_pose.pose.orientation<<std::endl;
    std::cout<<"Start_Pose_Position = \n"<<start_pose.pose.position<<std::endl;
    
    start_pose.pose.position.z -= 1.2;  //消除默认规划位置不一致的问题
    start_pose.pose.position.x += 1.3;
    start_pose.pose.position.y += 0.9;

    // start_pose.pose.position.x -= 0.1;
    // arm.setPoseTarget(start_pose);
    // arm.move();


    //设置允许的最大速度和加速度
    arm.setStartStateToCurrentState();
    // start_pose.pose.position.x = 0.257;
    // start_pose.pose.position.y = -0.441;
    // start_pose.pose.position.z = 0.311;

    // start_pose.position.z += 0.1;
    start_pose.pose.orientation.x = 0.599;
    start_pose.pose.orientation.y = -0.376;
    start_pose.pose.orientation.z = -0.355;
    start_pose.pose.orientation.w = 0.612;
    std::vector<double> joint_group_positions;
    joint_group_positions.push_back(-1.344);
    joint_group_positions.push_back(0.489);
    joint_group_positions.push_back(-0.977);
    joint_group_positions.push_back(0.0);
    joint_group_positions.push_back(0.0);
    joint_group_positions.push_back(0.0);
    joint_group_positions[0] = -1.344;
    joint_group_positions[1] = 0.489;
    joint_group_positions[2] = -0.977;
    joint_group_positions[3] = 0.0;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 0.0;

    arm.setJointValueTarget(joint_group_positions);


    std::cout<<"Target_Pose_Orientation = \n"<<start_pose.pose.orientation<<std::endl;
    std::cout<<"Target_Pose_Position = \n"<<start_pose.pose.position<<std::endl;
    // arm.setPoseTarget(start_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    arm.plan(plan1);
    arm.asyncExecute(plan1);
    sleep(0.5);
    arm.setStartStateToCurrentState();
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.3);

    sleep(1);
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    arm.plan(plan2);
    std::cout<<"SLOW DOWN!"<<std::endl;
    arm.asyncExecute(plan2);

    sleep(10);
    // arm.setMaxAccelerationScalingFactor(1);
    // arm.setMaxVelocityScalingFactor(1);
	// // waypoints.push_back(start_pose);
    // start_pose.pose.position.z -= 0.1;
    // arm.setPoseTarget(start_pose);
    // arm.move();

	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.005;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数


    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

	ros::shutdown(); 
	return 0;
}
