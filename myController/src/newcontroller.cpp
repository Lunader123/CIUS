// Author: JiaDong Lu
// *********************************************************************
// ********************************NOTE ********************************
// 船新的代码，替换了轨迹执行的方式
// *********************************************************************
// *********************************************************************

#include <ros/ros.h>
#include <ctime>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ctime>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <sensor_msgs/PointCloud2.h>
#include <myController/obstacle.h>
#include <math.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// 定义一些全局变量
bool whethergo = 1;
myController::obstacle obstacle;

// 全局变量指针集合
struct globals
{
    std::vector<double> *tdnmPtr;
    std::vector<double> *tn;
    std::vector<std::vector<double>> *positionsPtr;
    int *point_nPtr;
    double *posi_nowPtr;
    double *velo_former;
    double *time_from_startPtr;
    std::vector<double> *single_acc;
    moveit::planning_interface::MoveGroupInterface::Plan *plan; //  仅用判断是否碰撞   
    bool isExecute = 0;
}global_Ptrs;


int main(int argc, char *argv[])
{
    // 声明函数
    bool myExecute(moveit::planning_interface::MoveGroupInterface::Plan plan, ros::Publisher pub_joint[], ros::Publisher pub_velo[]);
    void obstacle_cb(const myController::obstacleConstPtr& input, moveit::planning_interface::MoveGroupInterface *arm, \
                moveit::planning_interface::MoveGroupInterface::Plan &plan_for_obstacle, \
                moveit::planning_interface::PlanningSceneInterface *planning_scene);


    ros::init(argc,argv,"talker");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(5);
	spinner.start();

    ros::Publisher pub_joint[6];
    for(int i=0; i<6; i++){
        pub_joint[i] = nh.advertise<std_msgs::Float64>("/jaka/joint"+std::to_string(i+1)+"_position_controller/command", 10);
    }
    
    ros::Publisher pub_velo[6];
    for(int i=0; i<6; i++){
        pub_velo[i] = nh.advertise<std_msgs::Float64>("/jaka/joint"+std::to_string(i+1)+"_position_controller/velocity", 10);
    }

    moveit::planning_interface::MoveGroupInterface arm("jaka");
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    //当运动规划失败后，允许重新规划
    arm.allowReplanning(false);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.001);

    //设置一开始返回初始点允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.8);

    ros::Rate r(5);

    // 订阅障碍物数据
    moveit::planning_interface::PlanningSceneInterface planning_scene; //环境创建
    std::cout<<"正在等待planning scene 生成中..."<<std::endl;
    sleep(5);

    moveit::planning_interface::MoveGroupInterface::Plan plan_for_obstacle;
    ros::Subscriber sub_obs = nh.subscribe<myController::obstacle>("obstacle_msg", 10, boost::bind(obstacle_cb, _1, &arm, plan_for_obstacle, &planning_scene));
    
    while(ros::ok){
        std::cout<<"准备前往目的地!"<<std::endl;
        arm.setNamedTarget("home");
        arm.setStartStateToCurrentState();
        // arm.setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        // sleep(1);
        arm.plan(plan1);
        std::cout<<"轨迹规划成功，轨迹中共有"<<plan1.trajectory_.joint_trajectory.points.size()<<"个点。"<<std::endl;

        // 先分析一下plan里面的点把
        bool ifsuccese = myExecute(plan1, pub_joint, pub_velo);

        while(ros::ok()){
            if(ifsuccese){
                global_Ptrs.isExecute = 0;
                break;
            }
            sleep(0.02);
        }

        std::cout<<"回home成功！"<<std::endl;

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
        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        arm.plan(plan2);
        bool ifsucces = myExecute(plan2, pub_joint, pub_velo);

        while(ros::ok()){
            if(ifsucces){
                global_Ptrs.isExecute = 0;
                break;
            }
            sleep(0.02);
        }
    }
    return 0;
}


bool myExecute(moveit::planning_interface::MoveGroupInterface::Plan plan, ros::Publisher pub_joint[], ros::Publisher pub_velo[])
{
    void getAccelaration(std::vector<std::vector<double>>& acce, std::vector<std::vector<double>> positions, std::vector<double> tdnm);
    std::vector<double> getSingleAcc(double posi_now[], std::vector<double> position, double velo_former[], double t);

    std::cout<<"开始执行规划轨迹"<<std::endl;
    // 确定发布频率 50 hz
    ros::Rate r(1);
    int count = 0;

    std::vector<double> tdnm;
    std::vector<double> tn;

    std::vector<std::vector<double>> acce;
    std::vector<std::vector<double>> velo;
    std::vector<std::vector<double>> positions;

    double time_former = 0;
    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan.trajectory_.joint_trajectory.points.begin(),
                                                                          end = plan.trajectory_.joint_trajectory.points.end(); via != end; ++via)
    {
        positions.push_back(via->positions); 
        tdnm.push_back(via->time_from_start.toSec() - time_former);
        tn.push_back(via->time_from_start.toSec());
        time_former = via->time_from_start.toSec();
    }

    std::vector<double>::const_iterator k = tdnm.begin();
    tdnm.erase(k); //删除第一个元素tnm=0

    std::cout<<"Point 1"<<std::endl;

    // 求加速度(已弃用)
    // getAccelaration(acce, positions, tdnm);

    std::cout<<"Point 2"<<std::endl;
    // 求速度
    ros::Rate rate(1000);
    double delta_t = 0.001;
    double time_from_start = 0;
    double posi_last[6];
    for(int i = 0; i < 6; i++){
        posi_last[i] = plan.trajectory_.joint_trajectory.points[0].positions[i];
    }

    double posi_now[6] = {0,0,0,0,0,0};
    double velo_former[6] = {0,0,0,0,0,0};
    int point_n = 1;

    // 首先获取初始加速度
    std::vector<double> single_acc = getSingleAcc(posi_last, positions[point_n], velo_former, tdnm[0]);

    //开始执行前global_Ptrs赋值
    global_Ptrs.point_nPtr = &point_n;
    global_Ptrs.posi_nowPtr = &posi_now[0];
    global_Ptrs.positionsPtr = &positions;
    global_Ptrs.single_acc = &single_acc;
    global_Ptrs.tdnmPtr = &tdnm;
    global_Ptrs.time_from_startPtr = &time_from_start;
    global_Ptrs.tn = &tn;
    global_Ptrs.velo_former = &velo_former[0];
    global_Ptrs.plan = &plan;
    global_Ptrs.isExecute = true;
    // std::cout<<"初始轨迹规划成功：输出Joint2的positions："<<std::endl;
    // for(int i = 0; i < global_Ptrs.positionsPtr->size();i++){
    //     std::cout<<i<<"  "<<(*global_Ptrs.positionsPtr)[i][1]<<std::endl;
    // }
    // std::cout<<"原始轨迹的tn是："<<std::endl;
    // for(int i = 0; i < global_Ptrs.tn->size();i++){
    //     std::cout<<i<<"  "<<(*global_Ptrs.tn)[i]<<std::endl;
    // }
    // std::cout<<"原始轨迹的tdnm是："<<std::endl;
    // for(int i = 0; i < global_Ptrs.tdnmPtr->size();i++){
    //     std::cout<<i<<"  "<<(*global_Ptrs.tdnmPtr)[i]<<std::endl;
    // }

    // std::cout<<global_Ptrs.posi_nowPtr[2]<<std::endl;

    while(ros::ok){
        // std::cout<<"Execute 1"<<std::endl;
        for(int i = 0; i<6; i++){
            posi_now[i] = posi_last[i] + velo_former[i] * delta_t + 0.5*single_acc[i]*delta_t*delta_t;
            posi_last[i] = posi_now[i];
            velo_former[i] = velo_former[i] + single_acc[i] * delta_t;
        }
        // std::cout<<"Execute 2"<<std::endl;
        for(int i = 0; i < 6; i++){
            std_msgs::Float64 posijoint;
            posijoint.data = posi_now[i];
            std_msgs::Float64 velojoint;
            velojoint.data = velo_former[i];
            pub_joint[i].publish(posijoint);
            pub_velo[i].publish(velojoint);
        }
        // std::cout<<"Execute 3"<<std::endl;
        time_from_start += delta_t;
        // std::cout<<"Point "<<point_n<< " 当前tn为："<< tn[point_n]<<std::endl;
        if(time_from_start > tn[point_n])
        {
            std::cout<<"当前实际上的位置是："<<posi_now[0]<<std::endl;
            std::cout<<"当前想要的位置是："<<positions[point_n][0]<<std::endl;
            point_n += 1;
            if(point_n >= positions.size()){
                break;
            }
            // std::cout<<"Point "<<point_n<< " 当前tn为："<< tn[point_n]<<std::endl;
            single_acc = getSingleAcc(posi_now, positions[point_n], velo_former, tdnm[point_n-1]);
            // std::cout<<"Execute 5"<<std::endl;
        }
        rate.sleep();
    }
    
    std::cout<<"Point 3"<<std::endl;

    return true;
}


std::vector<double> getSingleAcc(double posi_now[], std::vector<double> position, double velo_former[], double t){
    std::vector<double> acce_now;
    for(int i = 0; i < 6; i++){
        acce_now.push_back(2 * (position[i] - posi_now[i] - velo_former[i] * t) / (t * t));
    }
    return acce_now;
}

void getAccelaration(std::vector<std::vector<double>>& acce, std::vector<std::vector<double>> positions, std::vector<double> tdnm){
    // 求加速度
    clock_t startTime,endTime,midlleTime;
    startTime = clock();
    std::vector<double> posi_former = positions[0];
    std::vector<double> velo_former = {0,0,0,0,0,0};
    for(int n = 1; n < positions.size(); n++){
        std::vector<double> acce_now;
        for(int i = 0; i < 6; i++){
            acce_now.push_back(2 * (positions[n][i] - posi_former[i] - velo_former[i] * tdnm[n-1]) / (tdnm[n-1] * tdnm[n-1]));
            velo_former[i] += acce_now[i] * tdnm[n-1];
            posi_former[i] = positions[n][i];
        }
        acce.push_back(acce_now);
    }
    endTime = clock();
    std::cout << " 计算加速度用时："<<double(endTime - startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
}

void slowDonw();

void obstacle_cb(const myController::obstacleConstPtr& input, moveit::planning_interface::MoveGroupInterface *arm, \
                moveit::planning_interface::MoveGroupInterface::Plan &plan_for_obstacle,\
                moveit::planning_interface::PlanningSceneInterface *planning_scene)
{
    // ROS_INFO_STREAM("收到障碍物信息，Z方向上的均值是："<<input->z_max);
    obstacle = *input;
    whethergo = 0;

    // 首首先要判断现在是否在执行轨迹
    if(global_Ptrs.isExecute == 0){
        std::cout<<"现在没有轨迹在执行"<<std::endl;
        return;
    }

    double time_from_start = *global_Ptrs.time_from_startPtr;
    moveit::planning_interface::MoveGroupInterface::Plan plan = *global_Ptrs.plan;


    // 首先要判断障碍物是否会阻挡轨迹
    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan.trajectory_.joint_trajectory.points.begin(),
                                                                          end = plan.trajectory_.joint_trajectory.points.end(); via != end; ++via)
    {
        if(via->time_from_start.toSec() - time_from_start < 0)
        {
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
        if((distance < obstacle.z_max - obstacle.z_mean +0.07) || (distance < obstacle.z_mean - obstacle.z_min +0.07))
        // if(false)
        // if((distance < obstacle.z_max - obstacle.z_min + 0.04))
        {
            std::cout<<"障碍物已经入侵轨迹，准备重新规划"<<std::endl;
            moveit_msgs::CollisionObject collosion_object;
            collosion_object.header.frame_id = "base_link";
            
            double length = obstacle.z_max - obstacle.z_min;
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = length+0.1;
            primitive.dimensions[1] = length+0.1;
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
            planning_scene->addCollisionObjects(collosion_objects);
            
            arm->setStartStateToCurrentState();
            arm->plan(plan_for_obstacle);

            // 计算时间
            clock_t startTime,endTime,midlleTime;
            startTime = clock();

            int point_n = *global_Ptrs.point_nPtr;
            std::vector<double> tdnm = *global_Ptrs.tdnmPtr;
            std::vector<double> tn = *global_Ptrs.tn;
            std::vector<std::vector<double>> positions = *global_Ptrs.positionsPtr;
            // double time_from_start = *global_Ptrs.time_from_startPtr;
            double posi_now[6];
            double velo_former[6];
            for(int i = 0; i < 6; i++)
            {
                posi_now[i] = global_Ptrs.posi_nowPtr[i];
                velo_former[i] = global_Ptrs.velo_former[i];
            }


            std::vector<double> single_acc = *global_Ptrs.single_acc;
            point_n += 1;
            //  修改positions
            positions.erase(positions.begin() + point_n, positions.end());
            tdnm.erase(tdnm.begin() + point_n-1, tdnm.end());
            tn.erase(tn.begin() + point_n, tn.end());
            double time_former = time_from_start;
            double time_this_has_pass = time_from_start - tn.back();

            trajectory_msgs::JointTrajectoryPoint combine_point;
            std::vector<trajectory_msgs::JointTrajectoryPoint> combine_points;
            plan.trajectory_.joint_trajectory.points.erase(plan.trajectory_.joint_trajectory.points.begin()+point_n, plan.trajectory_.joint_trajectory.points.end());
            combine_points.insert(combine_points.end(),plan.trajectory_.joint_trajectory.points.begin(),plan.trajectory_.joint_trajectory.points.end());
            for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator via = plan_for_obstacle.trajectory_.joint_trajectory.points.begin()+1,
                                                                                end = plan_for_obstacle.trajectory_.joint_trajectory.points.end(); via != end; ++via)
            {
                positions.push_back(via->positions); 
                tn.push_back(via->time_from_start.toSec() + time_from_start);
                combine_point.positions = via->positions;
                combine_point.time_from_start = ros::Duration(via->time_from_start.toSec() + time_from_start);
                combine_points.push_back(combine_point);
                tdnm.push_back(tn.back() - time_former);
                time_former = tn.back();

            }
            // 修改tdnm

            single_acc = getSingleAcc(posi_now, positions[point_n], velo_former, tdnm[point_n-1]-time_this_has_pass);

            // 将修改传回去
            *global_Ptrs.single_acc = single_acc;
            *global_Ptrs.positionsPtr = positions;
            *global_Ptrs.tn = tn;
            *global_Ptrs.tdnmPtr = tdnm;
            *global_Ptrs.point_nPtr = point_n;
            plan.trajectory_.joint_trajectory.points = combine_points;
            *global_Ptrs.plan = plan;

            // *global_Ptrs.plan->trajectory_.joint_trajectory.points.erase(*global_Ptrs.plan->trajectory_.joint_trajectory.points.begin()+point_n,*global_Ptrs.plan->trajectory_.joint_trajectory.points.end());

            
            // plan.trajectory_.joint_trajectory.points


            std::cout<<"重新规划成功，已经重新发送轨迹"<<std::endl;
            // for(int i = 0; i < global_Ptrs.positionsPtr->size();i++){
            //     std::cout<<i<<"  "<<(*global_Ptrs.positionsPtr)[i][1]<<std::endl;
            // }
            // std::cout<<"重新规划后的tn是："<<std::endl;
            // for(int i = 0; i < global_Ptrs.tn->size();i++){
            //     std::cout<<i<<"  "<<(*global_Ptrs.tn)[i]<<std::endl;
            // }
            // std::cout<<"重新规划后的tdnm是："<<std::endl;
            // for(int i = 0; i < global_Ptrs.tdnmPtr->size();i++){
            //     std::cout<<i<<"  "<<(*global_Ptrs.tdnmPtr)[i]<<std::endl;
            // }
            // std::cout<<"重新规划后的points time是："<<std::endl;
            // for(int i = 0; i < global_Ptrs.plan->trajectory_.joint_trajectory.points.size();i++){
            //     std::cout<<i<<"  "<<(global_Ptrs.plan->trajectory_.joint_trajectory.points[i].time_from_start)<<std::endl;
            // }
            // std::cout<<"fuck"<<std::endl;
            endTime = clock();
            std::cout << " 计算加速度用时："<<double(endTime - startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
            break;
        }

    }
}