// Author: JiaDong Lu
// *********************************************************************
// ********************************NOTE ********************************
// 这个代码还包括很多问题，有时候运行并不稳定，目前推测是ros消息机制延迟带来的，
// chatter2不会在第一时间收到result，发出应该的标志位，将会在一周内改进升级。
// 目前的改进思路是不使用ros的消息机制，自己替换一下，但是也很有可能稍微动一下
// 代码就可以修复，还在研究ing...
// *********************************************************************
// *********************************************************************

#include <ros/ros.h>
#include <ctime>
#include <sstream>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ctime>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <sensor_msgs/PointCloud2.h>
#include <myController/obstacle.h>
#include <math.h>

bool whethergo = 1;
bool finishexe = 0;
bool nowIsReplan = 0;
bool ignortResultOnce = 0;

ros::Duration d(0.01);

void avoidObs(const myController::obstacleConstPtr& input);

myController::obstacle obstacle;

class Listener
{
    public:
    void chatter1(const std_msgs::String::ConstPtr& msg_p)
    {
        ROS_INFO_STREAM("我听到 ");
        d.sleep();
    }
    void chatter2(const control_msgs::FollowJointTrajectoryActionResultConstPtr& result)
    {
        if(ignortResultOnce)
        {
            ignortResultOnce = 0;
            std::cout<<"忽略发送finishexe一次"<<std::endl;
        }
        else{
            finishexe = 1;
            std::cout<<"发送finishexe一次"<<std::endl;
        }
        d.sleep();
    }
    void cloud_cb(const myController::obstacleConstPtr& input)
    {
        // ROS_INFO_STREAM("收到障碍物信息，Z方向上的均值是："<<input->z_max);
        obstacle = *input;
        whethergo = 0;
        d.sleep();
    }
};


int main(int argc, char *argv[])
{
    void callback(const std_msgs::String::ConstPtr& msg_p);

    setlocale(LC_ALL,"");

    ros::init(argc,argv,"talker");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(5);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("jaka");
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    //当运动规划失败后，允许重新规划
    arm.allowReplanning(false);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.001);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.4);
    arm.setMaxVelocityScalingFactor(0.2);

    moveit_msgs::CollisionObject collosion_object;
    collosion_object.header.frame_id = "base_link";
    moveit::planning_interface::PlanningSceneInterface planning_scene; //环境创建
    std::cout<<"正在等待planning scene 生成中..."<<std::endl;
    sleep(5);

    // 返回原点
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);


    Listener l;
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter", 10, &Listener::chatter1, &l);
    ros::Subscriber sub2 = nh.subscribe<control_msgs::FollowJointTrajectoryActionResult>\
                            ("/jaka/arm_joint_controller/follow_joint_trajectory/result", 1, &Listener::chatter2, &l);
    ros::Subscriber sub_obs = nh.subscribe<myController::obstacle>("obstacle_msg", 1, &Listener::cloud_cb, &l);

    // ros::AsyncSpinner s(3);
    // s.start();

    ros::Rate r(5);
    while(ros::ok())
    {
        // 清除障碍物这个函数现在是没有用的，还在思考为啥，这个函数到底怎么做
        // 添加个需要删除的障碍物名称，然后通过planning scene interface完成删除
        std::vector<std::string> object_ids;
        object_ids.push_back("haha");
        planning_scene.removeCollisionObjects(object_ids);
        
restart:
        std::cout<<"准备前往目的地!"<<std::endl;
        std::vector<double> joint_group_positions;
        joint_group_positions.push_back(-1.944);
        joint_group_positions.push_back(0.489);
        joint_group_positions.push_back(-0.977);
        joint_group_positions.push_back(0.0);
        joint_group_positions.push_back(0.0);
        joint_group_positions.push_back(0.0);
        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        arm.plan(plan1);
        arm.asyncExecute(plan1);
        finishexe = 0;

        // plan1.trajectory_.joint_trajectory.points
        // std::cout<<plan1.trajectory_.joint_trajectory.points.<<std::endl;

        while(ros::ok())
        {
            std::cout<<"while开始"<<std::endl;
            if(whethergo == 0)
            {
                //avoid_colli();
                // ROS_INFO_STREAM("MEET COLLISION!");
                std::cout<<"成功收到障碍物信息，将会研判障碍物是否阻碍轨迹，障碍物预估大小是："<<obstacle.z_max-obstacle.z_min<<std::endl;
                int count=0;
                for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan1.trajectory_.joint_trajectory.points.begin(),
                                                                          end = plan1.trajectory_.joint_trajectory.points.end(); via != end; ++via)
                {
                    count += 1;
                    if(count<=5){
                        continue;
                    }
                    // std::cout<<"现在关节的位置是："<<std::endl;
                    std::vector<double> joint_angles = via->positions;
                    double t1 =  via->positions[0];
                    double t2 =  via->positions[1];
                    double t3 =  via->positions[2];
                    double t4 =  via->positions[3];
                    double t5 =  via->positions[4];
                    double t6 =  via->positions[5];
                    double x = (353*cos(t1)*cos(t2))/1000 - (231*sin(t1))/2000 + (93*cos(t5)*sin(t1))/1000 + (41*cos(t4)*(cos(t1)*cos(t2)*sin(t3)\
                     + cos(t1)*cos(t3)*sin(t2)))/400 - (41*sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)))/400\
                     + (93*sin(t5)*(cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + sin(t4)*(cos(t1)*cos(t2)*sin(t3)\
                     + cos(t1)*cos(t3)*sin(t2))))/1000 - (303*cos(t1)*sin(t2)*sin(t3))/1000 + (303*cos(t1)*cos(t2)*cos(t3))/1000;
                    double y = (231*cos(t1))/2000 - (93*cos(t1)*cos(t5))/1000 + (353*cos(t2)*sin(t1))/1000 + (41*cos(t4)*(cos(t2)*sin(t1)*sin(t3)\
                     + cos(t3)*sin(t1)*sin(t2)))/400 - (41*sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)))/400 + (93*sin(t5)*(cos(t4)*(sin(t1)*sin(t2)*sin(t3)\
                     - cos(t2)*cos(t3)*sin(t1)) + sin(t4)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))))/1000 - (303*sin(t1)*sin(t2)*sin(t3))/1000 + (303*cos(t2)*cos(t3)*sin(t1))/1000;
                    double z = (353*sin(t2))/1000 + (303*cos(t2)*sin(t3))/1000 + (303*cos(t3)*sin(t2))/1000 - (93*sin(t5)*(cos(t4)*(cos(t2)*sin(t3) + cos(t3)*sin(t2))\
                     + sin(t4)*(cos(t2)*cos(t3) - sin(t2)*sin(t3))))/1000 - (41*cos(t4)*(cos(t2)*cos(t3) - sin(t2)*sin(t3)))/400 + (41*sin(t4)*(cos(t2)*sin(t3) + cos(t3)*sin(t2)))/400 + 57/500;
                    double distance = sqrt(pow((x - obstacle.x_mean), 2.0) + pow((y - obstacle.y_mean), 2.0) + pow((z - obstacle.z_mean), 2.0));
                    // std::cout<<"当前路径点到障碍物的距离是："<<distance<<std::endl;
                    if((distance < obstacle.z_max - obstacle.z_mean +0.12) || (distance < obstacle.z_mean - obstacle.z_min +0.12))
                    // if((distance < obstacle.z_max - obstacle.z_min + 0.04))
                    {
                        //此时需要重新规划
                        std::cout<<"障碍物会阻挡轨迹！将障碍物加入地图，重新开始规划！"<<std::endl;

                        double length = obstacle.z_max - obstacle.z_min;
                        shape_msgs::SolidPrimitive primitive;
                        primitive.type = primitive.BOX;
                        primitive.dimensions.resize(3);
                        primitive.dimensions[0] = length+0.10;
                        primitive.dimensions[1] = length+0.10;
                        primitive.dimensions[2] = length+0.15;

                        geometry_msgs::Pose box_pose;
                        box_pose.orientation.w = 1.0;
                        box_pose.position.x = obstacle.x_mean;
                        box_pose.position.y = obstacle.y_mean;
                        box_pose.position.z = obstacle.z_mean;

                        collosion_object.primitives.clear();
                        collosion_object.id = "haha";
                        collosion_object.primitives.push_back(primitive);
                        collosion_object.primitive_poses.push_back(box_pose);
                        collosion_object.operation = collosion_object.ADD;

                        std::vector<moveit_msgs::CollisionObject> collosion_objects;
                        collosion_objects.clear();
                        collosion_objects.push_back(collosion_object);
                        planning_scene.addCollisionObjects(collosion_objects);
                        whethergo = 1;
                        ignortResultOnce = 1;
                        goto restart;
                    }
                    
                }
                whethergo = 1;
                continue;
            }
            if(finishexe == 1)
            {
                std::cout<<"目标点成功到达目标点！"<<std::endl;
                finishexe = 0;
                break;
            }
            r.sleep();
        }
restart_home:
        std::cout<<"准备返回初始点！"<<std::endl;
        arm.setStartStateToCurrentState();
        arm.setNamedTarget("home");
        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        arm.plan(plan2);
        arm.asyncExecute(plan2);
        finishexe = 0;
        while(ros::ok())
        {
            if(whethergo == 0)
            {
                //avoid_colli();
                // ROS_INFO_STREAM("MEET COLLISION!");
                std::cout<<"成功收到障碍物信息，将会研判障碍物是否阻碍轨迹，障碍物预估大小是："<<obstacle.z_max-obstacle.z_min<<std::endl;

                int count = 0;
                for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan2.trajectory_.joint_trajectory.points.begin(),
                                                                          end = plan2.trajectory_.joint_trajectory.points.end(); via != end; ++via)
                {
                    count += 1;
                    if(count<=5){
                        continue;
                    }
                    // std::cout<<"现在关节的位置是："<<std::endl;
                    std::vector<double> joint_angles = via->positions;
                    double t1 =  via->positions[0];
                    double t2 =  via->positions[1];
                    double t3 =  via->positions[2];
                    double t4 =  via->positions[3];
                    double t5 =  via->positions[4];
                    double t6 =  via->positions[5];
                    double x = (353*cos(t1)*cos(t2))/1000 - (231*sin(t1))/2000 + (93*cos(t5)*sin(t1))/1000 + (41*cos(t4)*(cos(t1)*cos(t2)*sin(t3)\
                     + cos(t1)*cos(t3)*sin(t2)))/400 - (41*sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)))/400\
                     + (93*sin(t5)*(cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + sin(t4)*(cos(t1)*cos(t2)*sin(t3)\
                     + cos(t1)*cos(t3)*sin(t2))))/1000 - (303*cos(t1)*sin(t2)*sin(t3))/1000 + (303*cos(t1)*cos(t2)*cos(t3))/1000;
                    double y = (231*cos(t1))/2000 - (93*cos(t1)*cos(t5))/1000 + (353*cos(t2)*sin(t1))/1000 + (41*cos(t4)*(cos(t2)*sin(t1)*sin(t3)\
                     + cos(t3)*sin(t1)*sin(t2)))/400 - (41*sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1)))/400 + (93*sin(t5)*(cos(t4)*(sin(t1)*sin(t2)*sin(t3)\
                     - cos(t2)*cos(t3)*sin(t1)) + sin(t4)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))))/1000 - (303*sin(t1)*sin(t2)*sin(t3))/1000 + (303*cos(t2)*cos(t3)*sin(t1))/1000;
                    double z = (353*sin(t2))/1000 + (303*cos(t2)*sin(t3))/1000 + (303*cos(t3)*sin(t2))/1000 - (93*sin(t5)*(cos(t4)*(cos(t2)*sin(t3) + cos(t3)*sin(t2))\
                     + sin(t4)*(cos(t2)*cos(t3) - sin(t2)*sin(t3))))/1000 - (41*cos(t4)*(cos(t2)*cos(t3) - sin(t2)*sin(t3)))/400 + (41*sin(t4)*(cos(t2)*sin(t3) + cos(t3)*sin(t2)))/400 + 57/500;
                    double distance = sqrt(pow((x - obstacle.x_mean), 2.0) + pow((y - obstacle.y_mean), 2.0) + pow((z - obstacle.z_mean), 2.0));
                    // std::cout<<"当前路径点到障碍物的距离是："<<distance<<std::endl;
                    if((distance < obstacle.z_max - obstacle.z_mean + 0.12) || (distance < obstacle.z_mean - obstacle.z_min + 0.12))
                    // if((distance < obstacle.z_max - obstacle.z_min + 0.04))
                    {
                        //此时需要重新规划
                        std::cout<<"障碍物会阻挡轨迹！将障碍物加入地图，重新开始规划！"<<std::endl;

                        double length = obstacle.z_max - obstacle.z_min;
                        shape_msgs::SolidPrimitive primitive;
                        primitive.type = primitive.BOX;
                        primitive.dimensions.resize(3);
                        primitive.dimensions[0] = length+0.12;
                        primitive.dimensions[1] = length+0.12;
                        primitive.dimensions[2] = length+0.12;

                        geometry_msgs::Pose box_pose;
                        box_pose.orientation.w = 1.0;
                        box_pose.position.x = obstacle.x_mean;
                        box_pose.position.y = obstacle.y_mean;
                        box_pose.position.z = obstacle.z_mean;

                        collosion_object.primitives.clear();
                        collosion_object.id = "haha";
                        collosion_object.primitives.push_back(primitive);
                        collosion_object.primitive_poses.push_back(box_pose);
                        collosion_object.operation = collosion_object.ADD;

                        std::vector<moveit_msgs::CollisionObject> collosion_objects;
                        collosion_objects.clear();
                        collosion_objects.push_back(collosion_object);
                        planning_scene.addCollisionObjects(collosion_objects);

                        whethergo = 1;
                        ignortResultOnce = 1;
                        goto restart_home;
                    }
                    
                }
                whethergo = 1;
                continue;
            }
            if(finishexe == 1)
            {
                std::cout<<"初始点成功返回初始点！ "<<std::endl;
                finishexe = 0;
                break;
            }
            r.sleep();
        }   
    }
    return 0;
}
