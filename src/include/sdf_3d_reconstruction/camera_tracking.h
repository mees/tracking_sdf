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
	Eigen::Matrix<double,3,3> K;
public:
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
	void projectCamera3DPointToImagePlane(Vector3i& camera_point, Vector2i& image_point);
};

#endif /* CAMERA_TRACKING_H_ */