#include "sdf_3d_reconstruction/sdf_reconstruction.h"
#include <stdlib.h>

//todo: implement
float SDF_Reconstruction::projectivePointToPointDistance(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans, Vector3i &voxel_coordinates){
	return 0;
}


//void SDF_Reconstruction::kinect_callback(const sensor_msgs::ImageConstPtr& image_rgb,
//		const sensor_msgs::ImageConstPtr& image_depth) {
void SDF_Reconstruction::kinect_callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
	cout<<"callback!"<<endl;
	try {
		//listener.waitForTransform("/world", "/openni_rgb_optical_frame",
		//		ros::Time(), ros::Duration(12.0));
		listener.lookupTransform("/world", "/openni_rgb_optical_frame",
				ros::Time(), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}
	Matrix4f trans_matrix;
	sensor_msgs::PointCloud2 test_cloud;
	pcl_ros::transformAsMatrix(transform, trans_matrix);
	//cout<<"Transform Matrix: "<<trans_matrix<<endl;
	pcl_ros::transformPointCloud(trans_matrix, *ros_cloud, test_cloud);
	test_cloud.header.frame_id = "/world";
	
	Vector3d trans;
	Eigen::Quaterniond rot;
	//TODO: als tf belassen?
	tf::vectorTFToEigen(transform.getOrigin(),trans);
	tf::quaternionTFToEigen(transform.getRotation(), rot);

	Matrix3d rotMat = rot.toRotationMatrix();//quaternion.toRotationMatrix();
	this->camera_tracking->set_camera_transformation(rotMat, trans);
	tf::Quaternion q = transform.getRotation();
	tf::Vector3 v = transform.getOrigin();
	cout<<" stamp: "<<transform.stamp_<<endl;
	//cout<<"Transform Matrix: "<<trans_matrix<<endl;
	cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << endl;
	cout << "- Quat Rotation: [" << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() <<"]" << endl;
	pub.publish(test_cloud);
	//ros::Duration(1).sleep();
	//cout << "- Rotation Matrix: "<<rotMat<<endl;

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*ros_cloud, pcl_pc2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	//fast bilateral filter for smoothing depth information in organized point clouds
	pcl::FastBilateralFilter<pcl::PointXYZRGB> bFilter;
	bFilter.setInputCloud(pcl_cloud);
//	bFilter.setHalfSize(5.0f);
//	bFilter.setStdDev(0.2f);
	bFilter.applyFilter(*cloud_filtered);
	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud_filtered);
	ne.compute(*normals);
//	cv::Mat depth_map(cloud_filtered->height, cloud_filtered->width, CV_16UC1);
//	cv::imshow("foo", depth_map);
//	cv::waitKey(3);
	
	sdf->update(this->camera_tracking, cloud_filtered, normals);
}

SDF_Reconstruction::SDF_Reconstruction() {
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
//	kinect_pcl_sub.subscribe(nh, "/camera/rgb/points", 1);
//    kinect_depth_sub.subscribe(nh, "/camera/depth/image", 1);
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1),kinect_pcl_sub, kinect_depth_sub);
//    sync.registerCallback(boost::bind(&SDF_Reconstruction::kinect_callback, this, _1, _2));
  
        this->camera_tracking = new CameraTracking();
	pcl = nh.subscribe("/camera/rgb/points", 1, &SDF_Reconstruction::kinect_callback, this);
	this->camera_tracking->cam_info = nh.subscribe("/camera/rgb/camera_info", 1,
			&CameraTracking::camera_info_cb, this->camera_tracking);
	
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/our_output/", 1);
	//Ros::Publisher(topic n)
	Vector3d sdf_origin(-4, -4, -0.2);
	
		     //m , width, height, depth, treshold
	sdf = new SDF(140, 8.0, 8.0, 2.0, sdf_origin,0.3, 0.05);
	//sdf->create_circle(200, 0, 0.0, 0.0);
	//std::string visualeOutput;
	//ros::param::get("~visualOutput", visualeOutput);
	//sdf->visualize();

	ros::spin();
}
