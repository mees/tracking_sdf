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
	//step for estimating gradient of translation
	float v_h;
	//precalculation of 2*trans_h for central_difference
	float v_h2;
	//step for estimating gradient of rotation
	float w_h;
	//precalculation of 2*w_h for central_difference
	float w_h2;
	//maximum of gauss_newton step
	int gauss_newton_max_iteration;
	//do not stop gauss newton until twist difference is bigger than maximum_twist diff
	float maximum_twist_diff;
	//rotation differences by changing w_x in minus (m) or plus (p) direction
	Eigen::Matrix<double, 3, 3> Rotdiff,r1p,r1m,r2p,r2m,r3p,r3m;
	//translation difference by changing v_x in minus (m) or plus (p) direction
	Eigen::Vector3d t1p, t1m, t2p, t2m, t3p, t3m;
public:
	//rotation camera -> global
	Eigen::Matrix3d rot;
	//rotation global -> camera
	Eigen::Matrix3d rot_inv;
	//translation camera -> global
	Eigen::Vector3d trans;
	// translation global -> camera
	Eigen::Vector3d rot_inv_trans;
	/**
	 * inner camera parameter
	 *      [fx  0 cx]
	 *  K = [ 0 fy cy]
	 *	[ 0  0  1]
	 */
	Eigen::Matrix3d K;
	ros::Subscriber cam_info;
	
	bool isKFilled;
	/**
	 *  standard constructor
	 */
	CameraTracking(int gauss_newton_max_iteration, float maximum_twist_diff, float w_h, float v_h);
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
	void get_partial_derivative(SDF *sdf, Eigen::Vector3d& camera_point, Eigen::Matrix<double, 6, 1>& SDF_derivative, bool& is_interpolated, double& sdf_val);
	/**
	 * estimates the new position from the old one given the new sdf
	 */
	void estimate_new_position(SDF *sdf,pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* CAMERA_TRACKING_H_ */
