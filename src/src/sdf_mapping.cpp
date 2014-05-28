#include "sdf_3d_reconstruction/sdf_mapping.h"

//todo: validate!
Vector2i SDF_Mapping::project3DPointToImagePlane(Vector3i XYZPoint){
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

void SDF_Mapping::camera_info_cb(const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info)
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

////incomplete!
float SDF_Mapping::projectivePointToPointDistance(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans, grid_index &gi){
//	/*transfer vertex global coordinates to the local
//	coordinate frame of the camera, Eq. 24 */
	Vector3i globalCoord;
	globalCoord(0)=gi.i;
	globalCoord(1)=gi.j;
	globalCoord(2)=gi.k;
	Vector3i localCoord;
//	//fixme: make this work
//	//Vector3i localCoord(CamRot*(globalCoord-CamTrans));
//	//now project this point to the pixel Eq. 25
	Vector2i ij = project3DPointToImagePlane(localCoord);
//
//	//compute difference of the depth of the voxel and the
//	//observed depth at (i, j)
//	//float distance = sdf.D[old_sdf.get_array_index(gi)] - ;
	return 0;
}

void SDF_Mapping::updateSDF(Matrix<double, 3, 3> &CamRot,
		Vector3d &CamTrans) {
	if (!camera_matrix.isFilled) {
		cout << "Camera Matrix not received. Start rosbag file!" << endl;
		exit(0);
	} else {
		for (int i = 0; i < sdf->getNumberOfVoxels(); i++) {
			grid_index gi;
			sdf->get_grid_index(i,gi);
			float d_n = projectivePointToPointDistance(CamRot, CamTrans, gi);
		}

	}

}

void SDF_Mapping::kinect_callback(const sensor_msgs::ImageConstPtr& image_rgb,
		const sensor_msgs::ImageConstPtr& image_depth) {

	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform("/world", "/openni_rgb_optical_frame",
				ros::Time(), ros::Duration(2.0));
		listener.lookupTransform("/world", "/openni_rgb_optical_frame",
				ros::Time(), transform);

//		 std::cout.precision(3);
//		 std::cout.setf(std::ios::fixed,std::ios::floatfield);
//		 std::cout << "At time " << transform.stamp_.toSec() << std::endl;

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


SDF_Mapping::SDF_Mapping() {

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
			sensor_msgs::Image> MySyncPolicy;

	kinect_rgb_sub.subscribe(nh, "/camera/rgb/image_color", 1);
	kinect_depth_sub.subscribe(nh, "/camera/depth/image", 1);
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1),
			kinect_rgb_sub, kinect_depth_sub);
	sync.registerCallback(
			boost::bind(&SDF_Mapping::kinect_callback, this, _1, _2));
	camInfo = nh.subscribe("/camera/rgb/camera_info", 1,
			&SDF_Mapping::camera_info_cb, this);
	sdf = new SDF(111, 1.0, 1.0, 1.0);
	sdf->create_circle(0.2, 0.5, 0.5, 0.5);
	std::cout<<"circle created ..." << std::endl;
	std::string visualeOutput;
	ros::param::get("~visualOutput", visualeOutput);
	sdf->visualize(visualeOutput);
	std::cout<<"circle visualized ..." << std::endl;

	ros::spin();
}
