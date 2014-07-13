#include "sdf_3d_reconstruction/sdf_reconstruction.h"
#include <stdlib.h>




//void SDF_Reconstruction::kinect_callback(const sensor_msgs::ImageConstPtr& image_rgb,
//		const sensor_msgs::ImageConstPtr& image_depth) {
void SDF_Reconstruction::kinect_callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud) {
	cout<<"callback!"<<endl;
	frame_num++;
	try {
		//listener.waitForTransform("/world", "/openni_rgb_optical_frame",
		//		ros::Time(), ros::Duration(12.0));
		listener.lookupTransform("/world", "/openni_rgb_optical_frame",
				ros::Time(), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}
	CALLGRIND_START_INSTRUMENTATION;
//	Matrix4f trans_matrix;
//	sensor_msgs::PointCloud2 test_cloud;
//	pcl_ros::transformAsMatrix(transform, trans_matrix);
//	//cout<<"Transform Matrix: "<<trans_matrix<<endl;
//	pcl_ros::transformPointCloud(trans_matrix, *ros_cloud, test_cloud);
//	test_cloud.header.frame_id = "/world";
	
	
	
//	tf::Quaternion q = transform.getRotation();
//	tf::Vector3 v = transform.getOrigin();
//	cout<<" stamp: "<<transform.stamp_<<endl;
//	cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << endl;
//	cout << "- Quat Rotation: [" << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() <<"]" << endl;
	//pub.publish(test_cloud);


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

	Vector3d trans;
	Eigen::Quaterniond rot;
	//TODO: als tf belassen?
	tf::vectorTFToEigen(transform.getOrigin(),trans);
	tf::quaternionTFToEigen(transform.getRotation(), rot);
	if (frame_num > 1){
	    this->camera_tracking->estimate_new_position(sdf,cloud_filtered);
	}
	Matrix3d rotMat = rot.toRotationMatrix();//quaternion.toRotationMatrix();	
	this->camera_tracking->set_camera_transformation(rotMat, trans);
	
	sdf->update(this->camera_tracking, cloud_filtered, normals);
	//cout<<"finished updating"<<endl;
//	if(frame_num==5){
//		sdf->visualize();
//		frame_num = 0;
//	}
	
	CALLGRIND_STOP_INSTRUMENTATION;
	CALLGRIND_DUMP_STATS;
}

SDF_Reconstruction::SDF_Reconstruction() {

     this->camera_tracking = new CameraTracking();
	pcl = nh.subscribe("/camera/rgb/points", 1, &SDF_Reconstruction::kinect_callback, this);
	this->camera_tracking->cam_info = nh.subscribe("/camera/rgb/camera_info", 1,
			&CameraTracking::camera_info_cb, this->camera_tracking);
	frame_num = 0;
	//pub = nh.advertise<sensor_msgs::PointCloud2> ("/our_output/", 1);
	//Ros::Publisher(topic n)
	Vector3d sdf_origin(-3.0, -4.0, -1.0);
	
		     //m , width, height, depth, treshold
	sdf = new SDF(140, 6.0, 6.0, 4.0, sdf_origin,0.3, 0.05);
	//sdf->create_cuboid(-1.0, 1.0, 0.0, 0.1, 0.2, 0.8);
	
	//sdf->create_circle(2.0, 0, 0.0, 0.0);
	//std::string visualeOutput;
	//ros::param::get("~visualOutput", visualeOutput);
	//sdf->visualize();

	ros::spin();
}
