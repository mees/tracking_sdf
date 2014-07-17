#include "sdf_3d_reconstruction/camera_tracking.h"
#include "sdf_3d_reconstruction/sdf.h"
CameraTracking::CameraTracking(int gauss_newton_max_iteration,
		float maximum_twist_diff, float v_h, float w_h, SDF *sdf) {
	this->trans = Eigen::Vector3d(0.909536, -0.419546, 0.798641);
	this->rot = Eigen::Matrix3d();
	this->rot << -0.616896, -0.195746, -0.762314, 0.786585, -0.120238, -0.605662, 0.0268973, -0.973255, 0.228145;
	this->set_camera_transformation(rot, trans);
	this->maximum_twist_diff = maximum_twist_diff;
	this->gauss_newton_max_iteration = gauss_newton_max_iteration;
	this->v_h = v_h;
	this->w_h = w_h;
	this->v_h2 = 2 * v_h;
	this->w_h2 = 2 * w_h;
	this->v_h2_width = this->v_h2/sdf->m_div_width;
	//cout << "v_h2_width" <<v_h2_width << endl;
	this->v_h2_height = this->v_h2/sdf->m_div_height;
	//cout << "v_h2_height" <<this->v_h2_height << endl;
	this->v_h2_depth =this->v_h2/sdf->m_div_depth;
	//cout << "v_h2_depth" <<this->v_h2_depth << endl;
}
CameraTracking::~CameraTracking() {
}

void CameraTracking::camera_info_cb(
		const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info) {
	this->K(0, 0) = rgbd_camera_info->K[0];
	this->K(0, 1) = rgbd_camera_info->K[1];
	this->K(0, 2) = rgbd_camera_info->K[2];
	this->K(1, 0) = rgbd_camera_info->K[3];
	this->K(1, 1) = rgbd_camera_info->K[4];
	this->K(1, 2) = rgbd_camera_info->K[5];
	this->K(2, 0) = rgbd_camera_info->K[6];
	this->K(2, 1) = rgbd_camera_info->K[7];
	this->K(2, 2) = rgbd_camera_info->K[8];
	this->isKFilled = true;
	cout << "read Camera Matrix" << this->K << endl;
	this->cam_info.shutdown();
}
/*
 * projects a point from camera perspective to image plane
 */
void CameraTracking::project_camera_to_image_plane(Vector3d& camera_point,
		Vector2d& image_point) {
	//Eq. 2
	Vector3d ij;
	ij = this->K * camera_point;
	image_point(0) = ij(0) / ij(2);
	image_point(1) = ij(1) / ij(2);
}
/**
 * project a global point from world coordinates to camera coordinates
 */
void CameraTracking::project_world_to_camera(Vector3d& world_point,
		Vector3d& camera_point) {
	camera_point = this->rot_inv * world_point + this->rot_inv_trans;
}
void CameraTracking::project_camera_to_world(const Vector3d& camera_point,
		Vector3d& world_point) {
	world_point = this->rot * camera_point + this->trans;
}
void CameraTracking::set_camera_transformation(Eigen::Matrix3d& rot,
		Eigen::Vector3d& trans) {
	this->rot = rot;
	this->rot_inv = rot.inverse();
	this->trans = trans;
	this->rot_inv_trans = -1 * (rot_inv * trans);
}
void CameraTracking::estimate_new_position(const SDF *sdf,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud) {
	ros::Time t0 = ros::Time::now();
	
	//twist_diff = (v1,v2,v3,w1,w2,w3)
	 twist_diff = Eigen::Matrix<double, 6, 1>::Zero();
	int np = omp_get_max_threads();
	//cout << "Threads: " << np<< endl;
	A_array = new boost::shared_ptr<Eigen::Matrix<double, 6, 6> >[np];
	B_array = new boost::shared_ptr<Eigen::Matrix<double, 6, 1> >[np];
	omp_set_num_threads(np);
	Vector3d r1,r2,r3;
	bool stop = false;
	for (int g = 0; g < gauss_newton_max_iteration && !stop; g++) {
		//setting A and B to 0
		A = Eigen::Matrix<double, 6, 6>::Zero();
		b = Eigen::Matrix<double, 6, 1>::Zero();
		/*
		 * We want to calculate the partial derivative of our sdf 
		 * with respekt to the twist coordinates. So we build up
		 * the linearized version of R' = w*R => R+_w = (I+w) R 
		 * 
		 * R_w1+ = 1.0 0.0 0.0
		 *         0.0 1.0 -w1
		 *         0.0  w1 1.0    
		 */
		Rotdiff(0, 0) = 1.0;
		Rotdiff(0, 1) = 0.0;
		Rotdiff(0, 2) = 0.0;
		Rotdiff(1, 0) = 0.0;
		Rotdiff(1, 1) = 1.0;
		Rotdiff(1, 2) = -w_h;
		Rotdiff(2, 0) = 0.0;
		Rotdiff(2, 1) = w_h;
		Rotdiff(2, 2) = 1.0;
		r1p = Rotdiff * this->rot;
		/* 
		 * R_w1- = 1.0 0.0 0.0
		 *         0.0 1.0  w1
		 *         0.0 -w1 1.0    
		 */
		Rotdiff(1, 2) = w_h;
		Rotdiff(2, 1) = -w_h;
		r1m = Rotdiff * this->rot;
		/* 
		 * R_w2+ = 1.0 0.0  w2
		 *         0.0 1.0 0.0
		 *         -w2 0.0 1.0    
		 */
		Rotdiff(1, 2) = 0;
		Rotdiff(2, 1) = 0;
		Rotdiff(0, 2) = w_h;
		Rotdiff(2, 0) = -w_h;
		r2p = Rotdiff * this->rot;
		/* 
		 * R_w2- = 1.0 0.0 -w2
		 *         0.0 1.0 0.0
		 *          w2 0.0 1.0    
		 */
		Rotdiff(0, 2) = -w_h;
		Rotdiff(2, 0) = w_h;
		r2m = Rotdiff * this->rot;
		/* 
		 * R_w3+ = 1.0 -w3 0.0
		 *          w3 1.0 0.0
		 *         0.0 0.0 1.0    
		 */
		Rotdiff(0, 2) = 0;
		Rotdiff(2, 0) = 0;
		Rotdiff(0, 1) = -w_h;
		Rotdiff(1, 0) = w_h;
		r3p = Rotdiff * this->rot;
		/* 
		 * R_w3- = 1.0  w3 0.0
		 *         -w3 1.0 0.0
		 *         0.0 0.0 1.0    
		 */
		Rotdiff(0, 1) = w_h;
		Rotdiff(1, 0) = -w_h;
		r3m = Rotdiff * this->rot;
#pragma omp parallel
		{
			boost::shared_ptr<Eigen::Matrix<double, 6, 6> > A_ptr;
			boost::shared_ptr<Eigen::Matrix<double, 6, 1> > B_ptr;
			A_ptr = boost::make_shared<Eigen::Matrix<double, 6, 6> >();
			B_ptr = boost::make_shared<Eigen::Matrix<double, 6, 1> >();
			A_ptr->setZero();
			B_ptr->setZero();
			A_array[omp_get_thread_num()] = A_ptr;
			B_array[omp_get_thread_num()] = B_ptr;
			bool is_interpolated;
			Eigen::Matrix<double, 6, 1> SDF_derivative;
			double int_dist;
			Eigen::Vector3d camera_point;
#pragma omp  for
			//iterate all image points
			for (int i = 0; i < point_cloud->width; i+=3) {
				for (int j = 0; j < point_cloud->height; j+=3) {

					pcl::PointXYZRGB point;
					point = point_cloud->at(i, j);
					//does a good depth exist?
					if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
						continue;
					}

					camera_point(0) = point.x;
					camera_point(1) = point.y;
					camera_point(2) = point.z;
					//get SDF_derivative
					get_partial_derivative(sdf, camera_point, SDF_derivative, is_interpolated, int_dist);
					//we could calculate SDF_derivative at this point
					if (!is_interpolated) {
						continue;
					}
					*A_ptr = *A_ptr + (SDF_derivative * SDF_derivative.transpose());
					*B_ptr = *B_ptr + (int_dist * SDF_derivative);
				}
			}
		}
		for (int i = 0; i < np; i++) {
			A += *A_array[i];
			b += *B_array[i];
		}
		//calculate our optimized gradient
		twist_diff = A.inverse() * b;

		//start inverting error by linearizing inverted rotation error
		Rotdiff(0, 0) = 1.0;
		Rotdiff(0, 1) = twist_diff(5, 0);
		Rotdiff(0, 2) = -twist_diff(4, 0);
		Rotdiff(1, 0) = -twist_diff(5, 0);
		Rotdiff(1, 1) = 1.0;
		Rotdiff(1, 2) = twist_diff(3, 0);
		Rotdiff(2, 0) = twist_diff(4, 0);
		Rotdiff(2, 1) = -twist_diff(3, 0);
		Rotdiff(2, 2) = 1.0;

		//invert translate error
		Vector3d t_inv(twist_diff(0, 0), twist_diff(1, 0), twist_diff(2, 0));
		t_inv = Rotdiff * t_inv;
		this->trans(0) = this->trans(0) - t_inv(0);
		this->trans(1) = this->trans(1) - t_inv(1);
		this->trans(2) = this->trans(2) - t_inv(2);
		//invert rotation error
		this->rot = Rotdiff * this->rot;
		Eigen::Quaterniond rot2;
		rot2 = (this->rot);
		this->set_camera_transformation(this->rot, this->trans);
		if (twist_diff(0, 0) < maximum_twist_diff
				&& twist_diff(1, 0) < maximum_twist_diff
				&& twist_diff(2, 0) < maximum_twist_diff
				&& twist_diff(3, 0) < maximum_twist_diff
				&& twist_diff(4, 0) < maximum_twist_diff
				&& twist_diff(5, 0) < maximum_twist_diff) {
			cout << "STOP Gauss Newton at step: "<< g << endl;
			stop = true;
		}
		//reorthomolize rotation
		r1 = rot.block(0, 0, 3, 1);
		r2 = rot.block(0, 1, 3, 1);
		r3 = rot.block(0, 2, 3, 1);
		r1 = r1 / r1.norm();
		r2 = (r2 - (r1.dot(r2)) * r1) / r2.norm();
		r3 = (r3 - (r1.dot(r3)) * r1 - (r2.dot(r3)) * r2) / r3.norm();

		rot.block(0, 0, 3, 1) = r1;
		rot.block(0, 1, 3, 1) = r2;
		rot.block(0, 2, 3, 1) = r3;
	}
	std::cout << "camera estimation method: " << (ros::Time::now() - t0).toSec()<< std::endl;

}
void CameraTracking::get_partial_derivative(const SDF* sdf,
		const Eigen::Vector3d& camera_point, Eigen::Matrix<double, 6, 1>& SDF_derivative, bool& is_interpolated, double& sdf_val) {
	Vector3d current_world_point;
	Vector3d current_voxel_point;
	//we use central difference
	Vector3d plus_h_world_point;
	Vector3d minus_h_world_point;
	Vector3d plus_h_camera_point;
	Vector3d minus_h_camera_point;
	Vector3d plus_h_voxel_point;
	Vector3d minus_voxel_point;
	float plus_h_sdf_value;
	float minus_h_sdf_value;
	this->project_camera_to_world(camera_point, current_world_point);
	sdf->get_voxel_coordinates(current_world_point, current_voxel_point);
	if (current_voxel_point(0) < 0 || current_voxel_point(1) < 0
			|| current_voxel_point(2) < 0) {
		return;
	}
	if (current_voxel_point(0) >= sdf->m || current_voxel_point(1) >= sdf->m
			|| current_voxel_point(2) >= sdf->m) {
		return;
	}
	sdf_val = sdf->interpolate_distance(current_voxel_point, is_interpolated);
	if (!is_interpolated)
		return;
	
	//tx derivative
	plus_h_voxel_point = current_voxel_point;
	plus_h_voxel_point(0) += v_h;
	minus_voxel_point = current_voxel_point;
	minus_voxel_point(0) -= v_h;
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	SDF_derivative(0) = (plus_h_sdf_value - minus_h_sdf_value) / v_h2_width;

	//ty derivative
	plus_h_voxel_point = current_voxel_point;
	plus_h_voxel_point(1) += v_h;
	minus_voxel_point = current_voxel_point;
	minus_voxel_point(1) -= v_h;
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	SDF_derivative(1) = (plus_h_sdf_value - minus_h_sdf_value) / v_h2_height;

	//tz derivative 
	plus_h_voxel_point = current_voxel_point;
	plus_h_voxel_point(2) += v_h;
	minus_voxel_point = current_voxel_point;
	minus_voxel_point(2) -= v_h;
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	SDF_derivative(2) = (plus_h_sdf_value - minus_h_sdf_value) / v_h2_depth;

	//wx derivative
	plus_h_world_point = r1p * camera_point + this->trans;
	minus_h_world_point = r1m * camera_point + this->trans;
	sdf->get_voxel_coordinates(plus_h_world_point, plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point, minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	SDF_derivative(3) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));

	//wy derivative
	plus_h_world_point = r2p * camera_point + this->trans;
	minus_h_world_point = r2m * camera_point + this->trans;
	sdf->get_voxel_coordinates(plus_h_world_point, plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point, minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	SDF_derivative(4) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));

	//wz derivative
	plus_h_world_point = r3p * camera_point + this->trans;
	minus_h_world_point = r3m * camera_point + this->trans;
	sdf->get_voxel_coordinates(plus_h_world_point, plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point, minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point,
			is_interpolated);
	if (!is_interpolated)
		return;
	SDF_derivative(5) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));

}
