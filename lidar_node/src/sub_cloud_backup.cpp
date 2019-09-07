#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <iostream>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "lidar_node/rel_pos1.h"
#include "lidar_node/lidar_pos.h"
#include "math.h"

/*
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <sys/socket.h>
 #include <unistd.h>
 #include <netinet/in.h>
 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
*/
#include "geometry_msgs/Twist.h"
// #include "lidar_node/mul_veh.h"
// #include "lidar_node/velocity.h"
#include "nav_msgs/Odometry.h"

// lidar_node::mul_veh mul_veh ;
// lidar_node::velocity velocity;
int cluster1_num;
int cluster2_num; 
float thd= 0.8;
float gap[30];
lidar_node::rel_pos1 rel_pos1;
lidar_node::lidar_pos lidar_pos;
ros::Publisher pub_front_cluster;

ros::Publisher pub_back_cluster;
ros::Publisher pub_relative_position;
ros::Publisher pub_lidar_position;
using namespace std; 
using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;
MatrixXf Rotation(3,3);
void Initialize1(){

  for(int i=0; i<30; i++){
    rel_pos1.baselink_x[i]=0;
    rel_pos1.baselink_y[i]=0;
    rel_pos1.baselink_z[i]=0;
    rel_pos1.twodim_x[i]=0;
    rel_pos1.twodim_y[i]=0;
    rel_pos1.twodim_z[i]=0;
  }
  
  for(int i=0; i<30; i++){
    lidar_pos.lidarlink_x[i]=0;
    lidar_pos.lidarlink_y[i]=0;
    lidar_pos.lidarlink_z[i]=0;
  }

}
void Initialize2(){

  
  
  for(int j=0; j<60; j++){
    rel_pos1.baselink2_x[j]=0;
    rel_pos1.baselink2_y[j]=0;
    rel_pos1.baselink2_z[j]=0;
    rel_pos1.twodim2_x[j]=0;
    rel_pos1.twodim2_y[j]=0;
    rel_pos1.twodim2_z[j]=0;
  }
 
  for(int j=0; j<60; j++){
    lidar_pos.lidarlink2_x[j]=0;
    lidar_pos.lidarlink2_y[j]=0;
    lidar_pos.lidarlink2_z[j]=0;
  }
  for(int i=0; i<30; i++){
    gap[i]=100;
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
//std::cout << "Roll: " << roll << std::endl;
//std::cout << "Pitch: " << pitch << std::endl;
//std::cout << "Yaw: " << yaw << std::endl;
MatrixXf R(3,3);
R << 1, 0, 0,
     0, cos(roll), -sin(roll),
     0, sin(roll), cos(roll);
MatrixXf P(3,3);
P << cos(pitch), 0, -sin(pitch), 
     0, 1, 0,
     sin(pitch), 0, cos(pitch);
MatrixXf Y(3,3);
Y << cos(yaw + 2.75878), -sin(yaw+2.75878), 0,
     sin(yaw + 2.75878), cos(yaw + 2.75878), 0,
     0, 0, 1;    
MatrixXf Rot(3,3);
Rot = R*P;
Rotation = Rot;  
}


void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& input)
{
  Initialize1();
  sensor_msgs::PointCloud2 front_cluster;


  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg (*input, cloud);
  

 
    /// PassThrough filter

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer2 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    *cloud_pointer1 = cloud;

    pcl::PassThrough<pcl::PointXYZI> pass1;
    pass1.setInputCloud (cloud_pointer1);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (0, 60);
    pass1.filter(*cloud_filtered);

    /*pcl::PassThrough<pcl::PointXYZI> pass2;
    pass2.setInputCloud (cloud_pointer2);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-15, 1);
    pass2.filter (*cloud_filtered);*/


  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (1.5); // 
  ec.setMinClusterSize (2);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
 

  cluster1_num=cluster_indices.size();
 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_temp (new pcl::PointCloud<pcl::PointXYZI>);

  int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          //added - limits to cluster only the drone
          //if (cloud_filtered->points[*pit].x > 0 && cloud_filtered->points[*pit].y>0 && cloud_filtered->points[*pit].y<10){
            cloud_filtered->points[*pit].intensity=(float)(50*(j+1));
          //if(cloud_filtered->points[*pit].y<-0.2 && cloud_filtered->points[*pit].x>0 && cloud_filtered->points[*pit].y>-15){
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
          }
          //}
            //}



       if(j>28) break;
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      cloud_cluster->header = cloud_filtered->header;
      cloud_cluster_temp->header = cloud_cluster->header;

      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      *cloud_cluster_temp += *cloud_cluster;
      //pcl::concatenateFields(*cloud_cluster, *cloud_cluster_temp, *cloud_cluster_temp);
      
       pcl::PointXYZI minPt, maxPt;
  pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
  float avg_x = (maxPt.x + minPt.x) / 2;
  float avg_y = (maxPt.y + minPt.y) / 2;
  float avg_z = (maxPt.z + minPt.z) / 2;
  lidar_pos.lidarlink_x[j] = avg_x;
  lidar_pos.lidarlink_y[j] = avg_y;
  lidar_pos.lidarlink_z[j] = avg_z;
  rel_pos1.baselink_x[j]=avg_x+0.7;//base link
  rel_pos1.baselink_y[j]=-avg_y;
  rel_pos1.baselink_z[j]=2-avg_z; //reversed front lidar
 
  Vector3f b_link(avg_x+0.7, -avg_y, 2-avg_z);
  Vector3f twoD = Rotation* b_link;
  //std::cout << "Rotation: " << Rotation << std::endl;
  std::cout << "cluster_no.: " << j + 1 << std::endl;
  std::cout << "BASELINK [" << b_link(0,0) << " ,"<< b_link(1,0)<<" ,"<<b_link(2,0)<<"]"<<std::endl; 
  std::cout << "final coordinate [" <<twoD(0,0) <<" ," <<twoD(1,0)<<" ,"<< twoD(2,0)<<"]"<<std::endl;
  rel_pos1.twodim_x[j] = twoD(0,0);
  rel_pos1.twodim_y[j] = twoD(1,0);
  rel_pos1.twodim_z[j] = twoD(2,0);
   j++;
    }
  

  
  pcl::toROSMsg(*cloud_cluster_temp, front_cluster);
  front_cluster.header.frame_id = "lidar_wamv_link";
  /////// check this later
  //output.header.stamp = ros::Time::now();
  front_cluster.header.stamp = input->header.stamp;
  /////// check this later
  //output.header.stamp = ros::Time::now();
  ///////
  pub_front_cluster.publish (front_cluster);
  //pub_back_cluster.publish (mul_veh); //////////////////////////////////////////////////////////////////////
  //ROS_INFO("Subscribe OK\n") ;
 // if(fabs(avg_x) > 0.001 || fabs(avg_y) > 0.001 || fabs(avg_z) > 0.001){
  //    pub_back_cluster.publish (mul_veh);
      //pub_relative_position.publish(center);
}
void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& input)
{Initialize2();
sensor_msgs::PointCloud2 back_cluster;
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg (*input, cloud);
 
    /// PassThrough filter

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pointer2 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    *cloud_pointer1 = cloud;

    pcl::PassThrough<pcl::PointXYZI> pass1;
    pass1.setInputCloud (cloud_pointer1);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (0, 60);
    pass1.filter(*cloud_filtered);

    /*pcl::PassThrough<pcl::PointXYZI> pass2;
    pass2.setInputCloud (cloud_pointer2);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-15, 1);
    pass2.filter (*cloud_filtered);*/


  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (1.2); // 
  ec.setMinClusterSize (2);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
 

 // cluster2_num=cluster_indices.size();
 
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_temp (new pcl::PointCloud<pcl::PointXYZI>);
  
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          //added - limits to cluster only the drone
          //if (cloud_filtered->points[*pit].x > 0 && cloud_filtered->points[*pit].y>0 && cloud_filtered->points[*pit].y<10){
            cloud_filtered->points[*pit].intensity=(float)(20*(j+1));
          //if(cloud_filtered->points[*pit].y<-0.2 && cloud_filtered->points[*pit].x>0 && cloud_filtered->points[*pit].y>-15){
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
          //}
        }





      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      cloud_cluster->header = cloud_filtered->header;
      cloud_cluster_temp->header = cloud_cluster->header;

      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      
      //pcl::concatenateFields(*cloud_cluster, *cloud_cluster_temp, *cloud_cluster_temp);
      
      //pcl::toROSMsg(*cloud_cluster, output);
      //pub.publish (output);
       // Min Max algorithm
      if (j>55) break;
  pcl::PointXYZI minPt, maxPt;
  pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
  float avg_x = (maxPt.x + minPt.x) / 2;
  float avg_y = (maxPt.y + minPt.y) / 2;
  float avg_z = (maxPt.z + minPt.z) / 2;
  for(int i=0; i<cluster1_num; i++){
    gap[i]=sqrt(((avg_x+0.7)-rel_pos1.baselink_x[i])*((avg_x+0.7)-rel_pos1.baselink_x[i])+((avg_y)-rel_pos1.baselink_y[i])*((avg_y)-rel_pos1.baselink_y[i])+((avg_z+1.8)-rel_pos1.baselink_z[i])*((avg_z+1.8)-rel_pos1.baselink_z[i]));
  }
  float *min_gap = std::min_element(gap,gap+30);
  //std::cout <<"min gap:"<< *min_gap<<std::endl;
  if(*min_gap>thd){ 
    *cloud_cluster_temp += *cloud_cluster;
    lidar_pos.lidarlink2_x[j] = avg_x;
  lidar_pos.lidarlink2_y[j] = avg_y;
  lidar_pos.lidarlink2_z[j] = avg_z;
  rel_pos1.baselink2_x[j] = avg_x+0.7;
  rel_pos1.baselink2_y[j] = avg_y;
  rel_pos1.baselink2_z[j] = 1.8+avg_z;
  Vector3f b_link(avg_x+0.7, avg_y, 1.8+avg_z);
  Vector3f twoD = Rotation* b_link;
  std::cout << "cluster_no.: " << j + (cluster1_num+1) << std::endl;
  std::cout << "BASELINK [" << b_link(0,0) << " ,"<< b_link(1,0)<<" ,"<<b_link(2,0)<<"]"<<std::endl; 
  std::cout << "final coordinate [" <<twoD(0,0) <<" ," <<twoD(1,0)<<" ,"<< twoD(2,0)<<"]"<<std::endl;
  rel_pos1.twodim2_x[j] = twoD(0,0);
  rel_pos1.twodim2_y[j] = twoD(1,0);
  rel_pos1.twodim2_z[j] = twoD(2,0);
   j++;
 }
 
int sum= cluster1_num + j;
//std::cout << "cluster numbers:" << sum<< std::endl;
rel_pos1.numcluster=sum;
  
    }

  
  pcl::toROSMsg(*cloud_cluster_temp, back_cluster);
  back_cluster.header.frame_id = "lidar_wamv2_link";
  /////// check this later
  //output.header.stamp = ros::Time::now();
  back_cluster.header.stamp = input->header.stamp;
  ///////
  pub_back_cluster.publish (back_cluster);
  pub_lidar_position.publish(lidar_pos);
  pub_relative_position.publish(rel_pos1);
  //pub_back_cluster.publish (mul_veh); //////////////////////////////////////////////////////////////////////
  //ROS_INFO("Subscribe OK\n") ;
 // if(fabs(avg_x) > 0.001 || fabs(avg_y) > 0.001 || fabs(avg_z) > 0.001){
  //    pub_back_cluster.publish (mul_veh);
      //pub_relative_position.publish(center);
}



int main (int argc, char** argv)
{

 // Initialization() ;
  // Initialize ROS
  ros::init (argc, argv, "lidar_node");

  ROS_INFO("==============START================");
  ros::NodeHandle nh;
  //Initialize();
  pub_front_cluster = nh.advertise<sensor_msgs::PointCloud2> ("front_cluster", 1);
  pub_back_cluster = nh.advertise<sensor_msgs::PointCloud2> ("back_cluster", 1);
  pub_relative_position = nh.advertise<lidar_node::rel_pos1> ("relative_position", 1);
  pub_lidar_position = nh.advertise<lidar_node::lidar_pos> ("lidar_position", 1);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_imu_data = nh.subscribe ("/imu/data", 1, imu_coord);
  ros::Subscriber sub_lidar_wamv_points = nh.subscribe ("/lidar_wamv/points", 1, cloud_cb1);
  ros::Subscriber sub_lidar_wamv2_points = nh.subscribe ("/lidar_wamv2/points", 1, cloud_cb2);

  // Create a ROS publisher for the output point cloud
  



  ros::spin ();


//http://docs.ros.org/melodic/api/sensor_msgs/html/msg/RegionOfInterest.html
}
