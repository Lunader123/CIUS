#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// 预计下面是用于TEM的库
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mycontroller");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("jaka");

    arm.setGoalJointTolerance(0.001);

    double accScale = 0.5;
    double velScale = 0.6;
    arm.setMaxAccelerationScalingFactor(accScale);
    arm.setMaxVelocityScalingFactor(velScale);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 获取机器人的起始位置
    moveit::core::RobotStatePtr start_state(arm.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(arm.getName());

    std::vector<double> joint_group_positions;
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //设置第一个目标点
    joint_group_positions[0] = 0.6;  // radians
    arm.setJointValueTarget(joint_group_positions);

    // 计算第一条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan1);
    // arm.asyncExecute(plan1);
    arm.execute(plan1);

    joint_model_group = start_state->getJointModelGroup(arm.getName());    
    start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    arm.setStartState(*start_state);

    //设置第二个目标点
    joint_group_positions[0] = 1.2;  // radians
    joint_group_positions[1] = 0.5;  // radians
    arm.setJointValueTarget(joint_group_positions);

    // 计算第二条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = arm.plan(plan2);
    // arm.execute(plan2);
    arm.asyncExecute(plan2);
    std::cout<<"Start Execute!"<<std::endl;

    sleep(1);
    // joint_model_group = start_state->getJointModelGroup(arm.getName());    
    // start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    // arm.setStartState(*start_state);
    arm.setStartStateToCurrentState();
    joint_group_positions[0] = -1.2;  // radians
    joint_group_positions[1] = 0.5;  // radians
    arm.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    success = arm.plan(plan3);
    // arm.execute(plan2);
    arm.asyncExecute(plan3);
    std::cout<<"Start Execute!"<<std::endl;


    sleep(1);

    ROS_INFO("Finished");

    // // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // ros::shutdown(); 

    return 0;
}
