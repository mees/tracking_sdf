#include "sdf_3d_reconstruction/camera_tracking.h"
#include "sdf_3d_reconstruction/sdf.h"
CameraTracking::CameraTracking(){
  
}
CameraTracking::~CameraTracking(){
  
}

void CameraTracking::camera_info_cb(const sensor_msgs::CameraInfoConstPtr &rgbd_camera_info)
{
	this->K(0,0) = rgbd_camera_info->K[0];
	this->K(0,1) = rgbd_camera_info->K[1];
	this->K(0,2) = rgbd_camera_info->K[2];
	this->K(1,0) = rgbd_camera_info->K[3];
	this->K(1,1) = rgbd_camera_info->K[4];
	this->K(1,2) = rgbd_camera_info->K[5];
	this->K(2,0) = rgbd_camera_info->K[6];
	this->K(2,1) = rgbd_camera_info->K[7];
	this->K(2,2) = rgbd_camera_info->K[8];
	this->isKFilled = true;
	cout<<"read Camera Matrix"<<this->K<<endl;
	this->cam_info.shutdown();
}
void CameraTracking::project_camera_to_image_plane(Vector3d& camera_point, Vector2d& image_point){
	//Eq. 2
	Vector3d ij;
	ij = this->K * camera_point;
	image_point(0) = ij(0)/ij(2); 
	image_point(1) = ij(1)/ij(2); 
}
void CameraTracking::project_world_to_camera(Vector3d& world_point, Vector3d& camera_point){
	camera_point = this->rot_inv*world_point + this->rot_inv_trans;
}
void CameraTracking::project_camera_to_world(Vector3d& camera_point, Vector3d& world_point){
	world_point = this->rot*camera_point + this->trans;
}
void CameraTracking::set_camera_transformation(Eigen::Matrix3d& rot, Eigen::Vector3d& trans){
	this->rot = rot;
	this->rot_inv = rot.inverse();
	this->trans= trans;
	this->rot_inv_trans = -1* (rot_inv * trans);
}
//TODO precalculate 2*(sdf->m_div_width)
void CameraTracking::get_partial_derivative(SDF* sdf, Eigen::Vector3d& camera_point, Eigen::Matrix<float, 6, 1>& SDF_derivative){
	
	Vector3d current_world_point;
	//we use central difference
	Vector3d before_world_point;
	Vector3d behind_world_point;
	float before;
	float behind;
	float difference_size;
	
	this->project_camera_to_world(camera_point, current_world_point);
	//tx derivative
	before_world_point = Vector3d(current_world_point(0)-1, current_world_point(1), current_world_point(2));
	behind_world_point = Vector3d(current_world_point(0)+1, current_world_point(1), current_world_point(2));
	before = sdf->interpolate_distance(before_world_point);
	behind = sdf->interpolate_distance(behind_world_point);
	SDF_derivative(0) = (before - behind)/(2*(sdf->m_div_width));
	
	//ty derivative
	before_world_point = Vector3d(current_world_point(0), current_world_point(1)-1, current_world_point(2));
	behind_world_point = Vector3d(current_world_point(0), current_world_point(1)+1, current_world_point(2));
	before = sdf->interpolate_distance(before_world_point);
	behind = sdf->interpolate_distance(behind_world_point);
	SDF_derivative(1) = (before - behind)/(2*(sdf->m_div_height));
	
	//tz derivative 
	before_world_point = Vector3d(current_world_point(0), current_world_point(1), current_world_point(2)-1);
	behind_world_point = Vector3d(current_world_point(0), current_world_point(1), current_world_point(2)+1);
	before = sdf->interpolate_distance(before_world_point);
	behind = sdf->interpolate_distance(behind_world_point);
	SDF_derivative(2) = (before - behind)/(2*(sdf->m_div_depth));
	
	//wx derivative
	before_world_point = Vector3d(current_world_point(0), current_world_point(1)+camera_point(2), current_world_point(2)-camera_point(1));
	behind_world_point = Vector3d(current_world_point(0), current_world_point(1)-camera_point(2), current_world_point(2)+camera_point(1));
	
}
