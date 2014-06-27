#ifndef CAMERA_TRACKING_H_
#define CAMERA_TRACKING_H_
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<Eigen/Eigen>
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
	 */
	Eigen::Matrix3d rot;
	Eigen::Matrix3d rot_inv;
	/*
	 * current translation
	 */
	Eigen::Vector3d trans;
	Eigen::Vector3d rot_inv_trans;
public:
	Eigen::Matrix3d K;
	ros::Subscriber cam_info;
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
	void project_world_to_camera(Vector3d& world_point, Vector3d& image_point);
	/**
	 * set current transformation of the camera. 
	 */
	void set_camera_transformation(Eigen::Matrix<double,3,3> rot, Eigen::Vector3d trans);
};

#endif /* CAMERA_TRACKING_H_ */