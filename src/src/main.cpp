//#include "sdf_3d_reconstruction/main.h"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  std::cout<<"Hello World2"<<std::endl;
  sensor_msgs::PointCloud2 output;
  // Publish the data
  //pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sdf_3d_reconstruction");
  ros::NodeHandle nh;
  std::cout<<"Hello World1"<<std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}


