#include "sdf_3d_reconstruction/sdf_reconstruction.h"

//todo: validate!
Vector2i SDF_Reconstruction::project3DPointToImagePlane(Vector3i XYZPoint){
	//Eq. 2
	Vector2i ij;
	float fx = camera_matrix.K(0,0);
	float fy = camera_matrix.K(1,1);
	float cx = camera_matrix.K(0,2);
	float cy = camera_matrix.K(1,2);
	ij(0) = fx*(XYZPoint(0)/XYZPoint(2)) + cx;
	ij(1) = fy*(XYZPoint(1)/XYZPoint(2)) + cy;
	return ij;
}

void SDF_Reconstruction::camera_info_cb(const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info)
{
	camera_matrix.K(0,0) = rgbd_camera_info->K[0];
	camera_matrix.K(0,1) = rgbd_camera_info->K[1];
	camera_matrix.K(0,2) = rgbd_camera_info->K[2];
	camera_matrix.K(1,0) = rgbd_camera_info->K[3];
	camera_matrix.K(1,1) = rgbd_camera_info->K[4];
	camera_matrix.K(1,2) = rgbd_camera_info->K[5];
	camera_matrix.K(2,0) = rgbd_camera_info->K[6];
	camera_matrix.K(2,1) = rgbd_camera_info->K[7];
	camera_matrix.K(2,2) = rgbd_camera_info->K[8];
	camera_matrix.isFilled = true;
	cout<<"read Camera Matrix"<<endl;
	camInfo.shutdown();
}


float SDF_Reconstruction::projectivePointToPointDistance(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans, Vector3i &voxel_coordinates){

	return 0;
}

void SDF_Reconstruction::updateSDF(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans) {
	if (!camera_matrix.isFilled) {
		cout << "Camera Matrix not received. Start rosbag file!" << endl;
		exit(0);
	} else {
		for (int i = 0; i < sdf->getNumberOfVoxels(); i++) {
			Vector3i voxel_coordinates;
			sdf->get_voxel_coordinates(i,voxel_coordinates);
			float d_n = projectivePointToPointDistance(CamRot, CamTrans, voxel_coordinates);
		}

	}

}

void SDF_Reconstruction::visualizeRGBCloudWithNormalsPCL(const PointCloud<PointXYZRGB>::Ptr &pcl_cloud,const PointCloud<Normal>::Ptr &normals) {
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(pcl_cloud);
	//viewer.addPointCloud<pcl::PointXYZRGB>(pcl_cloud);
	 viewer.addPointCloud<PointXYZRGB> (pcl_cloud, rgb, "sample cloud");
//	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(pcl_cloud,
//			normals);
	   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	   viewer.addPointCloudNormals<PointXYZRGB, pcl::Normal> (pcl_cloud, normals, 10, 0.05, "normals");
	   viewer.initCameraParameters ();

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

//void SDF_Reconstruction::kinect_callback(const sensor_msgs::ImageConstPtr& image_rgb,
//		const sensor_msgs::ImageConstPtr& image_depth) {
void SDF_Reconstruction::kinect_callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {

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

	//visualizeRGBCloudWithNormalsPCL(cloud_filtered, normals);

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

	     Matrix<double, 3, 3> rotMat = rot.toRotationMatrix();//quaternion.toRotationMatrix();
	     //	     tf::Quaternion q = transform.getRotation();
	     	     tf::Vector3 v = transform.getOrigin();
		cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << endl;
//		cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
//		                  << q.getZ() << ", " << q.getW() << "]" << endl;
	     updateSDF(rotMat, trans);

	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
	}
}


SDF_Reconstruction::SDF_Reconstruction() {

	pcl = nh.subscribe("/camera/rgb/points", 1, &SDF_Reconstruction::kinect_callback, this);
	camInfo = nh.subscribe("/camera/rgb/camera_info", 1,
			&SDF_Reconstruction::camera_info_cb, this);
	sdf = new SDF(111, 1.0, 1.0, 1.0);
	sdf->create_circle(0.2, 0.5, 0.5, 0.5);
	std::cout<<"circle created ..." << std::endl;
	std::string visualeOutput;
	ros::param::get("~visualOutput", visualeOutput);
	sdf->visualize(visualeOutput);
	std::cout<<"circle visualized ..." << std::endl;

	ros::spin();
}
