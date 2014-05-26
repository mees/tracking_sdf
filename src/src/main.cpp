#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <sstream>
#include <string>
//#include "sdf.h"

using namespace std;
using namespace Eigen;

struct cameraMatrix{
/* Intrinsic camera matrix for the raw (distorted) images.
     [fx  0 cx]
 K = [ 0 fy cy]
     [ 0  0  1]
 Projects 3D points in the camera coordinate frame to 2D pixel
 coordinates using the focal lengths (fx, fy) and principal point
 (cx, cy).*/
	Matrix<double,3,3> K;
	//set true when
	bool isFilled;
};


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  std::cout<<"Hello World2"<<std::endl;
  sensor_msgs::PointCloud2 output;
  // Publish the data
  //pub.publish (output);
}

void camera_info_cb(const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info, cameraMatrix &CamMatrix)
{
	CamMatrix.K(0,0) = rgbd_camera_info->K[0];
	CamMatrix.K(0,1) = rgbd_camera_info->K[1];
	CamMatrix.K(0,2) = rgbd_camera_info->K[2];
	CamMatrix.K(1,0) = rgbd_camera_info->K[3];
	CamMatrix.K(1,1) = rgbd_camera_info->K[4];
	CamMatrix.K(1,2) = rgbd_camera_info->K[5];
	CamMatrix.K(2,0) = rgbd_camera_info->K[6];
	CamMatrix.K(2,1) = rgbd_camera_info->K[7];
	CamMatrix.K(2,2) = rgbd_camera_info->K[8];
	CamMatrix.isFilled = true;
	//cout<<"ccallback! "<<CamMatrix<<endl;


}

void readGroundTruth(std::string &filename) {
	std::ifstream infile(filename.c_str());
	std::string line;
	while (std::getline(infile, line))
	{
	    std::istringstream iss(line);

	    float timestamp, tx, ty, tz, qx, qy, qz, qw;
	    iss >> timestamp >> tx >> ty >> tz >> qx >> qy >>qz >>qw;
	    Quaternionf quaternion(qx, qy, qz, qw);
	    Matrix<float, 3, 3> rotMat = quaternion.toRotationMatrix();
	    Vector3f trans(tx,ty,tz);
//	    cout<<"trans: "<<trans<<endl;
//	    cout<<"rot: "<<rotMat<<endl;
	}
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sdf_3d_reconstruction");
  ros::NodeHandle nh;
  std::cout<<"Hello World1"<<std::endl;
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  cameraMatrix camera_matrix;
  camera_matrix.isFilled = false;
  camera_matrix.K = Matrix<double,3,3>::Zero();

  ros::Subscriber camInfo = nh.subscribe<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1, boost::bind(camera_info_cb, _1, camera_matrix));

  std::string groundTruthFile;

    if (ros::param::has("~groundTruthPath"))
    {
      ros::param::get("~groundTruthPath", groundTruthFile);
      cout<<"read groundTruthPath file: "<<groundTruthFile<<endl;
    }
    else
    {
      cout << "Couldn't locate  ground Truth file" << endl;
      exit(0);
    }
  readGroundTruth(groundTruthFile);

  //cout<<"camera Matrix: "<<camera_matrix.K<<endl;
  //rgbd_camera_info.shutdown();
  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}


