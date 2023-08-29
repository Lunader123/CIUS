// scr/example.cpp
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include<pcl/search/impl/kdtree.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_listener.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <ctime>
#include <pcl_try/obstacle.h>

ros::Publisher pub;
ros::Publisher pub_obs;

bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) 
{
  clock_t startTime,endTime,midlleTime;
  startTime = clock();
  Eigen::Translation3f tl_btol(
  transform.getOrigin().getX(), 
  transform.getOrigin().getY(), 
  transform.getOrigin().getZ());
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
  transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  endTime = clock();
  std::cout << "The calculate Transform time is:"<<double(endTime - startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
  return true;
}

bool FilterBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr& filted_pc)
{
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setMin(Eigen::Vector4f(0.1, -0.1, 0,  1));
  box_filter.setMax(Eigen::Vector4f(-0.1, 0.1, 0.25,1));
  box_filter.setNegative(true);
  box_filter.setInputCloud(transform_pc);
  box_filter.filter(*filted_pc);
  std::cout<<"Filted points:"<<filted_pc->size()<<std::endl;
  return true;
}

struct S_Point
{
	double x;
	double y;
	double z;
};
 
double DistanceOfPointToLine(S_Point* a, S_Point* b, S_Point* s) 
{ 
  // std::cout<<"a:"<<b->x<<b->y<<b->z<<std::endl;
	double ab = sqrt(pow((a->x - b->x), 2.0) + pow((a->y - b->y), 2.0) + pow((a->z - b->z), 2.0));
	double as = sqrt(pow((a->x - s->x), 2.0) + pow((a->y - s->y), 2.0) + pow((a->z - s->z), 2.0));
	double bs = sqrt(pow((s->x - b->x), 2.0) + pow((s->y - b->y), 2.0) + pow((s->z - b->z), 2.0));
	double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
	double sin_A = sqrt(1 - pow(cos_A, 2.0));
  // std::cout<<"ab:"<<ab<<"as:"<<as<<"bs:"<<bs<<"cosA:"<<cos_A<<std::endl;
	return as*sin_A; 
}

bool GivePosition(S_Point& link1_posi, S_Point& link2_posi, S_Point& link3_posi,S_Point& link3_posi_true,\
                 S_Point& link4_posi_true, S_Point& link4_posi, S_Point& link5_posi, S_Point& link6_posi, \
                  tf::StampedTransform& transform1, tf::StampedTransform& transform2, tf::StampedTransform& transform3,\
                 tf::StampedTransform& transform3_true,tf::StampedTransform& transform4_true, tf::StampedTransform& transform4,\
                  tf::StampedTransform& transform5, tf::StampedTransform& transform6)
{
  link1_posi.x = double(transform1.getOrigin().x());
  link1_posi.y = double(transform1.getOrigin().y());
  link1_posi.z = double(transform1.getOrigin().z());
  link2_posi.x = double(transform2.getOrigin().x());
  link2_posi.y = double(transform2.getOrigin().y());
  link2_posi.z = double(transform2.getOrigin().z());
  link3_posi.x = double(transform3.getOrigin().x());
  link3_posi.y = double(transform3.getOrigin().y());
  link3_posi.z = double(transform3.getOrigin().z());
  link3_posi_true.x = double(transform3_true.getOrigin().x());
  link3_posi_true.y = double(transform3_true.getOrigin().y());
  link3_posi_true.z = double(transform3_true.getOrigin().z());
  link4_posi_true.x = double(transform4_true.getOrigin().x());
  link4_posi_true.y = double(transform4_true.getOrigin().y());
  link4_posi_true.z = double(transform4_true.getOrigin().z());
  link4_posi.x = double(transform4.getOrigin().x());
  link4_posi.y = double(transform4.getOrigin().y());
  link4_posi.z = double(transform4.getOrigin().z());
  link5_posi.x = double(transform5.getOrigin().x());
  link5_posi.y = double(transform5.getOrigin().y());
  link5_posi.z = double(transform5.getOrigin().z());
  link6_posi.x = double(transform6.getOrigin().x());
  link6_posi.y = double(transform6.getOrigin().y());
  link6_posi.z = double(transform6.getOrigin().z());
  // std::cout<<"haha"<<link1_posi.x<<link1_posi.y<<link1_posi.z<<std::endl;
  return true;
}


// callback function
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  clock_t startTime,endTime,midlleTime,t1,t2,t3;
  startTime = clock();
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform transform1;
  tf::StampedTransform transform2;
  tf::StampedTransform transform3;
  tf::StampedTransform transform3_true;
  tf::StampedTransform transform4_true;
  tf::StampedTransform transform4;
  tf::StampedTransform transform5;
  tf::StampedTransform transform6;
  Eigen::Matrix4f transform_matrix;
  // ROS_INFO("haha");
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::fromROSMsg(*input, *pc_cloud);
  pc_cloud->is_dense = false;
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*pc_cloud, *pc_cloud2, mapping);

  // 下面是体素滤波
  vg.setInputCloud(pc_cloud2);
  vg.setLeafSize(0.03f, 0.03f, 0.03f);
  vg.filter(*voxel_pc);

  listener.waitForTransform("/base_link","/camera_depth_optical_frame",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/camera_depth_optical_frame",ros::Time(0),transform);
  listener.waitForTransform("/base_link","/link_1",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_1",ros::Time(0),transform1);
  // 注意2,3和4用的是fake link
  listener.waitForTransform("/base_link","/link_2_fake",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_2_fake",ros::Time(0),transform2);
  listener.waitForTransform("/base_link","/link_3_fake",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_3_fake",ros::Time(0),transform3);
  // 注意有一个true link3 link4
  listener.waitForTransform("/base_link","/link_3",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_3",ros::Time(0),transform3_true);
  listener.waitForTransform("/base_link","/link_4",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_4",ros::Time(0),transform4_true);
  listener.waitForTransform("/base_link","/link_4_fake",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_4_fake",ros::Time(0),transform4);
  listener.waitForTransform("/base_link","/link_5",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_5",ros::Time(0),transform5);
  listener.waitForTransform("/base_link","/link_6",ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/base_link","/link_6",ros::Time(0),transform6);

  S_Point link1_posi;
  S_Point link2_posi;
  S_Point link3_posi;
  S_Point link3_posi_true;
  S_Point link4_posi_true;
  S_Point link4_posi;
  S_Point link5_posi;
  S_Point link6_posi;
  
  // 事实证明这样子没问题，获取每个link的xyz
  GivePosition(link1_posi, link2_posi, link3_posi, link3_posi_true, link4_posi_true, link4_posi, link5_posi, link6_posi, \
              transform1, transform2, transform3, transform3_true, transform4_true, transform4, transform5, transform6);


  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr passed_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filted_pc(new pcl::PointCloud<pcl::PointXYZ>);
  // 进行点云旋转
  TransformToMatrix(transform, transform_matrix); //由tf计算旋转矩阵
  pcl::transformPointCloud(*voxel_pc,*transform_pc,transform_matrix);

  // 直通滤波
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(transform_pc);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1.0, -0.05);
  pass.setFilterLimitsNegative(true);
  pass.filter(*passed_pc);

  
  
  int count = 0; //统计一下要过滤的点的数量
  pcl::PointCloud<pcl::PointXYZ>::iterator index = passed_pc->begin();
  for(size_t i=0;i<passed_pc->size();++i)
  {
    S_Point c;
    c.x = passed_pc->points[i].x;
    c.y = passed_pc->points[i].y;
    c.z = passed_pc->points[i].z;
    if (c.x<link6_posi.x+0.05||c.y< link6_posi.y+0.05)
    {
      // 判断到1的距离
      S_Point a;
      a.x = 0.0;
      a.y = 0.0;
      a.z = 0.0;
      double distance = DistanceOfPointToLine(&a, &link1_posi, &c);
      // std::cout<<"DISTANCE:"<<distance<<std::endl;
      if( distance < 0.15)
      {
        count+=1;
        continue;
      }
      // 从link2到link3的部分
      a = link2_posi;
      distance = DistanceOfPointToLine(&a, &link3_posi, &c);
      if( distance < 0.15)
      {
        count+=1;
        continue;
      }
      a = link3_posi_true;
      distance = DistanceOfPointToLine(&a, &link4_posi, &c);
      if( distance < 0.15)
      {
        count+=1;
        continue;
      }
      a = link4_posi_true;
      distance = sqrt(pow((a.x - c.x), 2.0) + pow((a.y - c.y), 2.0) + pow((a.z - c.z), 2.0));
      if( distance < 0.22)
      {
        count+=1;
        continue;
      }
      filted_pc->push_back(passed_pc->points[i]);
    }
  }
  std::cout<< "TOTALLY REMOVE -> "<<count<<std::endl;

  // 下面是聚类分割部分，只有在有点的时候才聚类
  if(filted_pc->size() > 10)
  {
    std::cout<<"filted_PointCloud_Size:"<<filted_pc->size()<<std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filted_pc);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.06); //6cm
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(1300);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filted_pc);
    ec.extract(cluster_indices);  //聚类的索引保存在这里
    // std::cout<<"Cluster: "<<cluster_indices.size()<<std::endl;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      //创建临时保存点云族的点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      //创建计算均值所用的变量
      double mean_x;
      double stddev_x;
      double mean_y;
      double stddev_y;
      double mean_z;
      double stddev_z;
      std::vector<float> vec_x;
      std::vector<float> vec_y;
      std::vector<float> vec_z;
      //通过下标，逐个填充
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
      {
        cloud_cluster->points.push_back(filted_pc->points[*pit]); //*
        vec_x.push_back(filted_pc->points[*pit].x);
        vec_y.push_back(filted_pc->points[*pit].y);
        vec_z.push_back(filted_pc->points[*pit].z);
      }
      pcl::getMeanStd(vec_x, mean_x, stddev_x);
      pcl::getMeanStd(vec_y, mean_y, stddev_y);
      pcl::getMeanStd(vec_z, mean_z, stddev_z);

      std::cout << "当前聚类 "<<j<< "-> 点云X方向上的均值为："<<mean_x<<std::endl;
      std::cout << "当前聚类 "<<j<< "-> 点云Y方向上的均值为："<<mean_y<<std::endl;
      std::cout << "当前聚类 "<<j<< "-> 点云Z方向上的均值为："<<mean_z<<std::endl;
      pcl::PointXYZ min;//用于存放三个轴的最小值
      pcl::PointXYZ max;//用于存放三个轴的最大值
      pcl::getMinMax3D(*cloud_cluster,min,max);
      std::cout << "当前聚类 "<<j<< "-> 点云Z方向上的MAX为："<<max.z<<std::endl;
      std::cout << "当前聚类 "<<j<< "-> 点云Z方向上的MIN为："<<min.z<<std::endl;

      pcl_try::obstacle my_msg;
      my_msg.x_mean = mean_x;
      my_msg.y_mean = mean_y;
      my_msg.z_mean = mean_z;
      my_msg.z_max = max.z;
      my_msg.z_min = min.z;
      pub_obs.publish(my_msg);
      
  
      //设置点云属性
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

  
      std::cout << "当前聚类 "<<j<<" 包含的点云数量: " << cloud_cluster->points.size() << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      // writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
      j++;
      
    }
  }


  endTime = clock();
  std::cout << "点云处理部分总运行时间为:"<<double(endTime - startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
  // std::cout << "The transform time is:"<<double(endTime - midlleTime)/ CLOCKS_PER_SEC << "s" << std::endl;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*filted_pc, output);
  output.header.frame_id = "base_link";
  pub.publish(output);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // tf::TransformListener listener;
  // tf::StampedTransform transform;

  // // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);

  // // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filted_points", 1);
  pub_obs = nh.advertise<pcl_try::obstacle> ("obstacle_msg", 1000);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // // Spin
  ros::spin ();
}
