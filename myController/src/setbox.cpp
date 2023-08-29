#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <iostream>
#include <string>
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "move_gazebo_model");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_state_srv;
    gazebo_msgs::ModelState des_model_state;
    geometry_msgs::Twist twist;
    int ans;
    bool do_skew=false;
    int n_line = 0;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;
    double x_bias = -1.02;
    double y_bias = -0.03;
    double z_bias = 0.1;
    double x,y,z;
    double dx = 0.15;
    double dy = 0.1;
    double dz = 0.1;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = 1.0;
    pose.orientation= quat;
    des_model_state.model_name = "my_box";
    des_model_state.pose = pose;
    des_model_state.twist = twist;
    des_model_state.reference_frame = "world";
    double qx,qy,qz,qw;
    //do random displacements and skews
    //cout<<"do skews? (0,1): ";
    //cin>>ans;
    //if (ans) do_skew=true;
    do_skew=true;
    pose.position.x = -1.05;
    pose.position.y = -1.5;
    pose.position.z = 1.3;
    des_model_state.pose = pose;
    set_model_state_srv.request.model_state = des_model_state;
    client.call(set_model_state_srv);
    ros::spin();
    // while(ros::ok()) {
    //     qx = 0.0;
    //     qy = 0.0;
    //     qz = 0.0;
    //     qw = 1.0;
    //     if(n_line == 0){
    //         x = x + 0.1;
    //         y = 0.0;
    //         z = 0.25;
    //         if(x >= 10){
    //             n_line = 1;
    //         }
    //     }
    //     else if( n_line == 1){
    //         x = x ;
    //         y = y + 0.1;
    //         z = 0.25;
    //         if(y >= 10){
    //             n_line = 2;
    //         }
    //     }
    //     else if(n_line == 2){
    //         x = x - 0.1;
    //         y = y ;
    //         z = 0.25;
    //         if(x <= -10){
    //             n_line = 3;
    //         }
    //     }
    //     else if(n_line == 3){
    //         x = x;
    //         y = y - 0.1;
    //         z = 0.25;
    //         if(y <= -10){
    //             n_line = 4;
    //         }
    //     }
    //     else if(n_line == 4){
    //         x = x + 0.1;
    //         y = y;
    //         z = 0.25;
    //         if(x >= 10){
    //             n_line = 5;
    //         }
    //     }
    //     else if(n_line == 5){
    //         x = x;
    //         y = y + 0.1;
    //         z = 0.25;
    //         if(y >= 0){
    //             n_line = 1;
    //         }
    //     }
    //     double norm = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    //     quat.x = qx/norm;
    //     quat.y = qy/norm;
    //     quat.z = qz/norm;
    //     quat.w = qw/norm;

    //     cout<<"qx, qy, qz, qw= "<<quat.x<<", "<<quat.y<<", "<<quat.z<<", "<<quat.w<<endl;
    //     cout<<"x,y,z = "<<x<<", "<<y<<", "<<z<<endl;
    //     pose.orientation= quat;
    //     pose.position.x = x;
    //     pose.position.y = y;
    //     pose.position.z = z;
    //     des_model_state.pose = pose;
    //     set_model_state_srv.request.model_state = des_model_state;
    //     client.call(set_model_state_srv);
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // }
    return 0;
}

