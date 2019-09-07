#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <iostream>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "lidar_node/rel_pos1.h"
#include "lidar_node/task5.h"
#include "math.h"
#include <string.h>


#include "geometry_msgs/Twist.h"

#include "nav_msgs/Odometry.h"
#define _USE_MATH_DEFINES

int cluster_num;
sensor_msgs::PointCloud2 front_cluster;
//sensor_msgs::PointCloud2 back_cluster;
lidar_node::rel_pos1 rel_pos1;
lidar_node::task5 task5;
ros::Publisher pub_front_cluster;
//ros::Publisher pub_back_cluster;
ros::Publisher pub_relative_position;
ros::Publisher pub_plane;
ros::Time stamp;
using namespace std; 
using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;

Eigen::Matrix4d transition1;
Eigen::Matrix4d transition2;
Eigen::Matrix3d RP_transform;
pcl::PointCloud<pcl::PointXYZI>::Ptr baselink_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr baselink_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr twoD_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr twoD_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final_baselink_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr final_twoD_cloud(new pcl::PointCloud<pcl::PointXYZI>);
void Initialize(){

  for(int i=0; i<30; i++){
    rel_pos1.baselink_x[i]=0;
    rel_pos1.baselink_y[i]=0;
    rel_pos1.baselink_z[i]=0;
    rel_pos1.twodim_x[i]=0;
    rel_pos1.twodim_y[i]=0;
    rel_pos1.twodim_z[i]=0;
  }

  for(int i=0; i<5; i++){
    task5.baselink_plane_x[i]=0;
    task5.baselink_plane_y[i]=0;
    task5.baselink_plane_z[i]=0;
    task5.twodim_plane_x[i]=0;
    task5.twodim_plane_y[i]=0;
    task5.twodim_plane_z[i]=0;

  }  
}

 
 


void imu_coord(const sensor_msgs::Imu::ConstPtr& msg){ //coordinate syncronize
  
  tf::Quaternion q(
  msg -> orientation.x,
  msg -> orientation.y,
  msg -> orientation.z,
  msg -> orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw); //RPY

  Eigen::Matrix3d Roll_transform = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Pitch_transform = Eigen::Matrix3d::Identity();
  transition1 = Eigen::Matrix4d::Identity();
  transition2 = Eigen::Matrix4d::Identity();


  Roll_transform(1,1) = cos(roll);
  Roll_transform(1,2) = -sin(roll);
  Roll_transform(1,1) = sin(roll);
  Roll_transform(1,1) = cos(roll);
  Pitch_transform(0,0) = cos(pitch);
  Pitch_transform(0,2) = -sin(pitch);
  Pitch_transform(2,0) = sin(pitch);
  Pitch_transform(2,2) = cos(pitch);
  RP_transform = Roll_transform*Pitch_transform;

  transition1(1,1) = -1;
  transition1(2,2) = -1;
  transition1(0,3) = 0.75;
  transition1(1,3) = 0.06;
  transition1(2,3) = 2;

  transition2(0,0) = cos(-M_PI/8);
  transition2(0,2) = -sin(-M_PI/8);
  transition2(2,0) = sin(-M_PI/8);
  transition2(2,2) = cos(-M_PI/8);
  transition2(0,3) = 0.75;
  transition2(1,3) = -0.06;
  transition2(2,3) = 2;

}


void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& input){ //Collect data from 32ch lidar
  Initialize();



  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg (*input, cloud);



  /// PassThrough filter

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer1 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer2 (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  *cloud_pointer1 = cloud;

  pcl::PassThrough<pcl::PointXYZI> pass1;
  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass1.setInputCloud (cloud_pointer1);
  pass1.setFilterFieldName ("x");
  pass1.setFilterLimits (0.5, 60);
  pass1.filter(*cloud_filtered);

  pcl::transformPointCloud (*cloud_filtered, *baselink_cloud1, transition1);
  pass2.setInputCloud (baselink_cloud1);
  pass2.setFilterFieldName ("z");
  pass2.setFilterLimits (0.7, 100);
  pass2.filter(*baselink_cloud1);
  *final_baselink_cloud += *baselink_cloud1;

}
void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& input){ // Collect data of 16ch lidar

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg (*input, cloud);

  /// PassThrough filter

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer1 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer2 (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  *cloud_pointer1 = cloud;

  pcl::PassThrough<pcl::PointXYZI> pass1;
  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass1.setInputCloud (cloud_pointer1);
  pass1.setFilterFieldName ("x");
  pass1.setFilterLimits (0.5, 60);
  pass1.filter(*cloud_filtered);

  pcl::transformPointCloud (*cloud_filtered, *baselink_cloud2, transition2);
  pass2.setInputCloud (baselink_cloud2);
  pass2.setFilterFieldName ("z");
  pass2.setFilterLimits (0.7, 100);
  pass2.filter(*baselink_cloud2);

  *final_baselink_cloud += *baselink_cloud2;
}




int main (int argc, char** argv)
{

// Initialization() ;
// Initialize ROS
ros::init (argc, argv, "docker_node");
ROS_INFO("==============START================");
ros::NodeHandle nh;
pub_front_cluster = nh.advertise<sensor_msgs::PointCloud2> ("front_cluster", 1);
//pub_back_cluster = nh.advertise<sensor_msgs::PointCloud2> ("back_cluster", 1);
pub_relative_position = nh.advertise<lidar_node::rel_pos1> ("relative_position", 1);
pub_plane = nh.advertise<lidar_node::task5> ("plane", 1);
ros::Subscriber sub_imu_data = nh.subscribe ("/imu/data", 1, imu_coord);
ros::Subscriber sub_lidar_wamv_points = nh.subscribe ("/lidar_wamv/points", 1, cloud_cb1);
ros::Subscriber sub_lidar_wamv2_points = nh.subscribe ("/lidar_wamv2/points", 1, cloud_cb2);

ros::Rate loop_rate(10);


while(ros::ok()){
// Create a ROS subscriber for the input point cloud
  Initialize();
  // Create a ROS publisher for the output point cloud
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (final_baselink_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  ec.setClusterTolerance (1.5); 
  ec.setMinClusterSize (2);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (final_baselink_cloud);
  ec.extract (cluster_indices);
  //cluster_num = cluster_indices.size();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_temp (new pcl::PointCloud<pcl::PointXYZI>);
  int j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // for single cluster
  {

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ //for single point in a cluster
  
    final_baselink_cloud->points[*pit].intensity=(double)(50*(j+1));

    cloud_cluster->points.push_back (final_baselink_cloud->points[*pit]);
    }





  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  cloud_cluster->header = final_baselink_cloud->header;
  cloud_cluster_temp->header = cloud_cluster->header;

  //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

  *cloud_cluster_temp += *cloud_cluster;
  //pcl::concatenateFields(*cloud_cluster, *cloud_cluster_temp, *cloud_cluster_temp);

  pcl::PointXYZI minPt, maxPt;
  pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
  double avg_x = (maxPt.x + minPt.x) / 2;
  double avg_y = (maxPt.y + minPt.y) / 2;
  double avg_z = (maxPt.z + minPt.z) / 2; 

  Vector3d b_link(avg_x, avg_y, avg_z);
  Vector3d twoD = RP_transform* b_link;
  size_t num_points = final_baselink_cloud->size(); 
  
  std::cout << "cluster_no.: " << j + 1 << std::endl;
 
  double twoD_distance = sqrt(twoD(0,0)*twoD(0,0)+twoD(1,0)*twoD(1,0)+twoD(2,0)*twoD(2,0));
  double twoD_bearing = atan(twoD(1,0)/twoD(0,0));
  twoD_distance +=0.1;
  twoD(0,0) = twoD_distance*cos(twoD_bearing);
  twoD(1,0) = twoD_distance*sin(twoD_bearing);
  rel_pos1.twodim_x[j] = twoD(0,0);
  rel_pos1.twodim_y[j] = twoD(1,0);
  rel_pos1.twodim_z[j] = twoD(2,0); // 2D coordinate of cluster
  rel_pos1.numcluster=cluster_indices.size();

  std::cout << "final coordinate [" <<twoD(0,0) <<" ," <<twoD(1,0)<<" ,"<< twoD(2,0)<<"]"<<std::endl;
  b_link = RP_transform.inverse()*twoD;
  std::cout << "Final BASELINK [" << b_link(0,0) << " ,"<< b_link(1,0)<<" ,"<<b_link(2,0)<<"]"<<std::endl; 
  rel_pos1.baselink_x[j]=b_link(0,0);
  rel_pos1.baselink_y[j]=b_link(1,0);
  rel_pos1.baselink_z[j]=b_link(2,0); // baselink coordinate of cluster
  if(j<5){
    task5.baselink_plane_x[j]=twoD(0,0);
    task5.baselink_plane_y[j]=twoD(1,0);
    task5.baselink_plane_z[j]=twoD(2,0);
    task5.twodim_plane_x[j]=b_link(0,0); 
    task5.twodim_plane_y[j]=b_link(1,0); 
    task5.twodim_plane_z[j]=b_link(2,0); 

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (1.5);
    seg.setInputCloud (cloud_cluster);
    seg.segment (*inliers, *coefficients);
    double a =coefficients -> values[0]/coefficients -> values[3];
    double b =coefficients -> values[1]/coefficients -> values[3];
    double c =coefficients -> values[2]/coefficients -> values[3]; // Normalizing: ax+by+cz+1=0
    std::cerr << "Plane equation: "<<a <<"x+"<<b<<"y+"<<c<<"z+"<<1<<"=0"<<std::endl; // plane equation of dockers
    task5.plane_a[j]=a;
    task5.plane_b[j]=b;
    task5.plane_c[j]=c;
    task5.plane_d[j]=1; // Plane equation coefficients
    }
  j++;
  }

  *final_baselink_cloud = {};
  pcl::toROSMsg(*cloud_cluster_temp, front_cluster);
  front_cluster.header.frame_id = "base_link";
  front_cluster.header.stamp = ros::Time::now();
  /////// check this later
  //output.header.stamp = ros::Time::now();
  //back_cluster.header.stamp = input->header.stamp;
  ///////
  pub_front_cluster.publish (front_cluster);
  pub_plane.publish(task5);
  pub_relative_position.publish(rel_pos1);

  loop_rate.sleep();
  ros::spinOnce();

  }

  //http://docs.ros.org/melodic/api/sensor_msgs/html/msg/RegionOfInterest.html
 // ros::spin();
}

