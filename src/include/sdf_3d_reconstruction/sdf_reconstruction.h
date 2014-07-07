#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>

#include "sdf_3d_reconstruction/sdf.h"
#include "sdf_3d_reconstruction/camera_tracking.h"
#include <fstream>
#include <sstream>
#include <string>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf_conversions/tf_eigen.h"
//#include "ros/ros.h"

using namespace Eigen;
using namespace pcl;
using namespace std;

class SDF_Reconstruction {

private:
		ros::NodeHandle nh;
		//cameraMatrix camera_matrix;
		CameraTracking *camera_tracking;
		SDF *sdf;
		message_filters::Subscriber<sensor_msgs::PointCloud2> kinect_pcl_sub;
		message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub;
		ros::Subscriber pcl;
		tf::TransformListener listener;
		tf::StampedTransform transform;
		ros::Publisher pub;

protected:
		//void camera_info_cb(const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info);
		//void kinect_callback(const sensor_msgs::ImageConstPtr& image_rgb, const sensor_msgs::ImageConstPtr& image_depth);
		void kinect_callback(const sensor_msgs::PointCloud2ConstPtr& pointCloud);

		float projectivePointToPointDistance(Matrix<double, 3, 3> &CamRot,
				Vector3d &CamTrans, Vector3i &gi);
		Vector2i project3DPointToImagePlane(Vector3i XYZPoint);
		void visualizeRGBCloudWithNormalsPCL(const PointCloud<PointXYZRGB>::Ptr &pcl_cloud, const PointCloud<Normal>::Ptr &normals);

public:
		SDF_Reconstruction();

};
