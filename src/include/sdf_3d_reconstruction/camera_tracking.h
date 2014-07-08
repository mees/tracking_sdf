#ifndef CAMERA_TRACKING_H_
#define CAMERA_TRACKING_H_
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<Eigen/Eigen>
class SDF;
//#include "sdf_3d_reconstruction/sdf.h"
using namespace Eigen;
using namespace std;
class CameraTracking {
  
private:
	/**
	 * inner camera parameter
	 *      [fx  0 cx]
	 *  K = [ 0 fy cy]
	 *	[ 0  0  1]
	 */
	/*
	 * current rotation matrix
	 * from Camera to World
	 */
	Eigen::Matrix3d rot;
	Eigen::Matrix3d rot_inv;
	Eigen::Vector3d rot_inv_trans;
public:
	Eigen::Matrix3d K;
	ros::Subscriber cam_info;
	/*
	 * current translation
	 */
	Eigen::Vector3d trans;
	bool isKFilled;
	/**
	 *  standard constructor
	 */
	CameraTracking();
	virtual ~CameraTracking();
	/**
	 * read the camera info
	 */
	void camera_info_cb(const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info);
	/*
	 * projects a point from camera perspective to image plane
	 */
	void project_camera_to_image_plane(Vector3d& camera_point, Vector2d& image_point);
	/**
	 * project a global point from world coordinates to camera coordinates
	 */
	void project_world_to_camera(Vector3d& world_point, Vector3d& camera_point);
	/**
	 * project a point from camera coordinates to world coordinates
	 */
	void project_camera_to_world(Vector3d& camera_point, Vector3d& world_point);
	/**
	 * set current transformation of the camera. 
	 */
	void set_camera_transformation(Eigen::Matrix3d& rot, Eigen::Vector3d& trans);
	/**
	 * calculate the current derivate of the SDF at point in camera coordinates
	 * xi is our current camera pose, where, you have to set it first
	 * rot = 
	 *    0 -w3  w2
	 *   w3   0 -w1 
	 *  -w2  w1   0 
	 * trans =
	 *   v1
	 *   v2
	 *   v3
	 */
	void get_partial_derivative(SDF *sdf, Eigen::Vector3d& camera_point,  Eigen::Matrix<float, 6, 1>& SDF_derivative);

public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* CAMERA_TRACKING_H_ */
