#include "sdf_3d_reconstruction/sdf_reconstruction.h"

//todo: implement
float SDF_Reconstruction::projectivePointToPointDistance(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans, Vector3i &voxel_coordinates){
	return 0;
}

void SDF_Reconstruction::updateSDF(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans) {
	if (this->camera_tracking->isKFilled) {
		cout << "Camera Matrix not received. Start rosbag file!" << endl;
		exit(0);
	} else {
		for (int i = 0; i < sdf->get_number_of_voxels(); i++) {
			Vector3i voxel_coordinates;
			sdf->get_voxel_coordinates(i,voxel_coordinates);
			float d_n = projectivePointToPointDistance(CamRot, CamTrans, voxel_coordinates);
		}
	}
}


//void SDF_Reconstruction::kinect_callback(const sensor_msgs::ImageConstPtr& image_rgb,
//		const sensor_msgs::ImageConstPtr& image_depth) {
void SDF_Reconstruction::kinect_callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud,const sensor_msgs::ImageConstPtr& image_depth) {
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
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform("/world", "/openni_rgb_optical_frame",
				ros::Time(), ros::Duration(2.0));
		listener.lookupTransform("/world", "/openni_rgb_optical_frame",
				ros::Time(), transform);
		Vector3d trans;
		Eigen::Quaterniond rot;
		tf::vectorTFToEigen(transform.getOrigin(),trans);
		tf::quaternionTFToEigen(transform.getRotation(), rot);
		Matrix3d rotMat = rot.toRotationMatrix();//quaternion.toRotationMatrix();
		this->camera_tracking->set_camera_transformation(rotMat, trans);
		//tf::Quaternion q = transform.getRotation();
		tf::Vector3 v = transform.getOrigin();
		cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << endl;
		updateSDF(rotMat, trans);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
	}
}

SDF_Reconstruction::SDF_Reconstruction() {
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
	kinect_pcl_sub.subscribe(nh, "/camera/rgb/points", 1);
    kinect_depth_sub.subscribe(nh, "/camera/depth/image", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1),kinect_pcl_sub, kinect_depth_sub);
    sync.registerCallback(boost::bind(&SDF_Reconstruction::kinect_callback, this, _1, _2));


        this->camera_tracking = new CameraTracking();
	//pcl = nh.subscribe("/camera/rgb/points", 1, &SDF_Reconstruction::kinect_callback, this);
	this->camera_tracking->cam_info = nh.subscribe("/camera/rgb/camera_info", 1,
			&CameraTracking::camera_info_cb, this->camera_tracking);
	sdf = new SDF(102, 200.0, 200.0, 200.0);
	sdf->create_circle(200, 0, 0.0, 0.0);
	std::string visualeOutput;
	ros::param::get("~visualOutput", visualeOutput);
	sdf->visualize(visualeOutput);

	ros::spin();
}
