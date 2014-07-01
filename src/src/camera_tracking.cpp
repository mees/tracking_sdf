#include "sdf_3d_reconstruction/camera_tracking.h"
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
//todo: validate!
void CameraTracking::project_camera_to_image_plane(Vector3d& camera_point, Vector2d& image_point){
	//Eq. 2
	Vector3d ij;
	/*float fx = this->K(0,0);
	float fy = this->K(1,1);
	float cx = this->K(0,2);
	float cy = this->K(1,2);*/
	ij = this->K * camera_point;
	image_point(0) = ij(0)/ij(2); //fx*(camera_point(0)/camera_point(2)) + cx;
	image_point(1) = ij(1)/ij(2); //fy*(camera_point(1)/camera_point(2)) + cy;
}
//todo: validate
void CameraTracking::project_world_to_camera(Vector3d& world_point, Vector3d& camera_point){
      camera_point = this->rot*world_point + this->rot_inv_trans;
	//camera_point(0) = this->rot_inv(0,0) * world_point(0) + this->rot_inv(0,1) * world_point(1) + this->rot_inv(0,2) * world_point(2) + this->rot_inv_trans(0);
	//camera_point(1) = this->rot_inv(1,0) * world_point(0) + this->rot_inv(1,1) * world_point(1) + this->rot_inv(1,2) * world_point(2) + this->rot_inv_trans(1);
	//camera_point(2) = this->rot_inv(2,0) * world_point(0) + this->rot_inv(2,1) * world_point(1) + this->rot_inv(2,2) * world_point(2) + this->rot_inv_trans(2);
}
//todo validate
void CameraTracking::set_camera_transformation(Eigen::Matrix3d rot, Eigen::Vector3d trans){
	this->rot = rot;
	this->rot_inv = rot.inverse();
	this->trans= trans;
	this->rot_inv_trans = -1* (rot_inv * trans);
}