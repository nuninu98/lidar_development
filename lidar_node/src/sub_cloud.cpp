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
#include "math.h"
#include <string.h>
#include <Eigen/Core>
#include <algorithm>    // std::sort
#include <vector>
#include "vrx_gazebo/Task.h"
#include <fstream>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#define _USE_MATH_DEFINES

int cluster_num;
//std::string task_msg;
sensor_msgs::PointCloud2 front_cluster;
//sensor_msgs::PointCloud2 back_cluster;
lidar_node::rel_pos1 rel_pos1;
vrx_gazebo::Task Task;

ros::Publisher pub_front_cluster;
//ros::Publisher pub_back_cluster;
ros::Publisher pub_relative_position;

ros::Time stamp;
using namespace std; 
using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;
using namespace Eigen;

Eigen::Matrix4f transition1;
Eigen::Matrix4f transition2;
Eigen::Matrix4f z_erase;
Eigen::Matrix3f RP_transform;
pcl::PointCloud<pcl::PointXYZI>::Ptr baselink_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr baselink_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr twoD_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr twoD_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final_baselink_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr final_z_erase(new pcl::PointCloud<pcl::PointXYZI>);
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
  
 

}

bool compare_head(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs){ //Matrix sort
    return lhs(0) > rhs(0);
}
 


void imu_coord(const sensor_msgs::Imu::ConstPtr& msg){ //coordinate rotation considering roll and ptch
  
  tf::Quaternion q(
    msg -> orientation.x,
    msg -> orientation.y,
    msg -> orientation.z,
    msg -> orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  Eigen::Matrix3f Roll_transform = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f Pitch_transform = Eigen::Matrix3f::Identity();
  transition1 = Eigen::Matrix4f::Identity();
  transition2 = Eigen::Matrix4f::Identity();
  z_erase = Eigen::Matrix4f::Identity();
  
  Roll_transform(1,1) = cos(roll);
  Roll_transform(1,2) = -sin(roll);
  Roll_transform(1,1) = sin(roll);
  Roll_transform(1,1) = cos(roll); // Roll rotational transform

  Pitch_transform(0,0) = cos(pitch);
  Pitch_transform(0,2) = -sin(pitch);
  Pitch_transform(2,0) = sin(pitch);
  Pitch_transform(2,2) = cos(pitch); // Pitch rotational transform
  RP_transform = Roll_transform*Pitch_transform;
 
  transition1(1,1) = -1;
  transition1(2,2) = -1;
  transition1(0,3) = 0.75;
  transition1(1,3) = 0.06;
  transition1(2,3) = 2; // 32ch lidar transition

  transition2(0,0) = cos(-M_PI/8);
  transition2(0,2) = -sin(-M_PI/8);
  transition2(2,0) = sin(-M_PI/8);
  transition2(2,2) = cos(-M_PI/8);
  transition2(0,3) = 0.75;
  transition2(1,3) = -0.06;
  transition2(2,3) = 2;// 16ch lidar transition

  z_erase(2,2) = 0; 
  

}

// void task_det(const vrx_gazebo::Task& task){
//   task_msg = task.name;
// }


void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& input) //collect data from 32ch lidar
{

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
;

  }
  void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& input) // collect data from 16ch lidar
{


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

}



int main (int argc, char** argv)
{
  

  ros::init (argc, argv, "lidar_node");
  ROS_INFO("==============START================");
  ros::NodeHandle nh;
  pub_front_cluster = nh.advertise<sensor_msgs::PointCloud2> ("front_cluster", 1);
  pub_relative_position = nh.advertise<lidar_node::rel_pos1> ("relative_position", 1);
  ros::Subscriber sub_imu_data = nh.subscribe ("/imu/data", 1, imu_coord);
  ros::Subscriber sub_lidar_wamv_points = nh.subscribe ("/lidar_wamv/points", 1, cloud_cb1);
  ros::Subscriber sub_lidar_wamv2_points = nh.subscribe ("/lidar_wamv2/points", 1, cloud_cb2);


  ros::Rate loop_rate(10);


  while(ros::ok()){

    *final_baselink_cloud += *baselink_cloud1;
    *final_baselink_cloud += *baselink_cloud2; //final_basekink_cloud: Collected raw data from both lidars
    ros::Time begin; 
    begin = ros::Time::now();
    // Create a ROS subscriber for the input point cloud
    Eigen::MatrixXd Lidar_Cartesian, Lidar_Cartesian_erase;
    Lidar_Cartesian.resize(final_baselink_cloud -> size(),2);

    Initialize();
    for(int i=0; i< final_baselink_cloud -> size(); i++){ // Save 2D distances and z coordinates of points in final_baselink_cloud into Lidar_cartesian
      Lidar_Cartesian(i,0) = sqrt(pow(final_baselink_cloud->points[i].x, 2)+pow(final_baselink_cloud->points[i].y, 2));
      Lidar_Cartesian(i,1) = final_baselink_cloud->points[i].z;
    }
    std::vector<Eigen::VectorXd> vec;
    for (int i = 0; i < Lidar_Cartesian.rows(); ++i){
      vec.push_back(Lidar_Cartesian.row(i));
    }
    std::sort(vec.begin(), vec.end(), &compare_head);  // Descending sort: Lidar_Cartesian's rows about  2D distances of each points  
    for (int i = 0; i < Lidar_Cartesian.rows(); ++i){
     Lidar_Cartesian.row(i) = vec[i];
    }

    // Create a ROS publisher for the output point cloud
    pcl::transformPointCloud (*final_baselink_cloud, *final_z_erase, z_erase); // Project points in final_baselink_cloud on xy plane(set z coordinates to 0)
    Lidar_Cartesian_erase= MatrixXd::Zero(final_z_erase -> size(),2);
    std::vector<pcl::PointIndices> cluster_indices;

    if(final_z_erase -> size()){
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud (final_z_erase);       
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance (0.3); //Euclidean clustering for final_z_erase
      ec.setMinClusterSize (2);
      ec.setMaxClusterSize (10000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (final_z_erase);
      ec.extract (cluster_indices);
    }
    else{
        std::cout << "no cluster" <<std::endl;
        rel_pos1.numcluster = 0;
      }



    //cluster_num = cluster_indices.size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_temp (new pcl::PointCloud<pcl::PointXYZI>); // To save total clusters
    int j=0;
 
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) // For each clusters
    {
     
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ //In single cluster, for each points   
          Lidar_Cartesian_erase(*pit, 0) = sqrt(pow(final_z_erase -> points[*pit].x, 2)+pow(final_z_erase -> points[*pit].y, 2)); //Lidar_cartesian_erase: save distances of each projected points on xy plane
          Lidar_Cartesian_erase(*pit, 1) = *pit;// Also save each point's index
      }


    } 

      
    std::vector<Eigen::VectorXd> vec2;
    for (int i = 0; i < Lidar_Cartesian_erase.rows(); ++i){
      vec2.push_back(Lidar_Cartesian_erase.row(i));
    }
    std::sort(vec2.begin(), vec2.end(), &compare_head);    
    for (int i = 0; i < Lidar_Cartesian_erase.rows(); ++i){ // Sort Lidar_cartesian_erase
      Lidar_Cartesian_erase.row(i) = vec2[i];
    }

    

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){


      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (int i = 0; i < Lidar_Cartesian_erase.rows(); i++){ 
      
        final_z_erase -> points[Lidar_Cartesian_erase(i,1)].z = Lidar_Cartesian(i,1); // Reset the z coordinate values for points in final_z_erase
  
                
       }
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      final_z_erase->points[*pit].intensity = (double)(20*(j+1));
      cloud_cluster->points.push_back(final_z_erase->points[*pit]); // cloud_cluster: cluster for single object
   
      }

      

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      cloud_cluster->header = final_z_erase->header;
      cloud_cluster_temp->header = cloud_cluster->header;

      *cloud_cluster_temp += *cloud_cluster; // cloud_cluster_temp: total group of clusters 


      pcl::PointXYZI minPt, maxPt;
      pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
      double avg_x = (maxPt.x + minPt.x) / 2;
      double avg_y = (maxPt.y + minPt.y) / 2;
      double avg_z = (maxPt.z + minPt.z) / 2;


      Vector3f b_link(avg_x, avg_y, avg_z);
      Vector3f twoD = RP_transform* b_link; //Rotational transform
      size_t num_points = final_baselink_cloud->size(); 
      double twoD_distance = sqrt(twoD(0,0)*twoD(0,0)+twoD(1,0)*twoD(1,0)+twoD(2,0)*twoD(2,0));
      double twoD_bearing = atan2(twoD(1,0),twoD(0,0));
      twoD_distance +=0.08;
      twoD(0,0) = twoD_distance*cos(twoD_bearing);
      twoD(1,0) = twoD_distance*sin(twoD_bearing); //Sensor error correction
    
        if(j<30){
          std::cout << "cluster_no.: " << j + 1 << std::endl;
          rel_pos1.twodim_x[j] = twoD(0,0);
          rel_pos1.twodim_y[j] = twoD(1,0);
          rel_pos1.twodim_z[j] = twoD(2,0);
          rel_pos1.numcluster=cluster_indices.size();
          std::cout << "final coordinate [" <<twoD(0,0) <<" ," <<twoD(1,0)<<" ,"<< twoD(2,0)<<"]"<<std::endl;
          b_link = RP_transform.inverse()*twoD;
          std::cout << "Final BASELINK [" << b_link(0,0) << " ,"<< b_link(1,0)<<" ,"<<b_link(2,0)<<"]"<<std::endl; 
          rel_pos1.baselink_x[j]=b_link(0,0);
          rel_pos1.baselink_y[j]=b_link(1,0);
          rel_pos1.baselink_z[j]=b_link(2,0); 
        }
      
      
      j++;


     


    }
    cluster_indices = {};
   
    



    *final_baselink_cloud = {};
    *final_z_erase = {};
    pcl::toROSMsg(*cloud_cluster_temp, front_cluster);
    front_cluster.header.frame_id = "base_link";
    front_cluster.header.stamp = ros::Time::now();
    /////// check this later
    //output.header.stamp = ros::Time::now();
    //back_cluster.header.stamp = input->header.stamp;
    ///////
    pub_front_cluster.publish (front_cluster);

    pub_relative_position.publish(rel_pos1);
    ros::Time end;
    end = ros::Time::now();
    std::cout<<"Time spend: "<<end-begin<<std::endl;

    loop_rate.sleep();
    ros::spinOnce();

  }// ros ok
} // main

