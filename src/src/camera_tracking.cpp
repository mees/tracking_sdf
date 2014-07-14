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
/*
 * projects a point from camera perspective to image plane
 */
void CameraTracking::project_camera_to_image_plane(Vector3d& camera_point, Vector2d& image_point){
	//Eq. 2
	Vector3d ij;
	ij = this->K * camera_point;
	image_point(0) = ij(0)/ij(2); 
	image_point(1) = ij(1)/ij(2); 
}
/**
 * project a global point from world coordinates to camera coordinates
 */
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
void CameraTracking::estimate_new_position(SDF *sdf,pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud){
	int i,j;
	Eigen::Matrix<double, 6, 1> SDF_derivative;
	
	pcl::PointXYZRGB point;
	Eigen::Vector3d camera_point;
	cout << "estimate" <<endl;
	double w_h = 0.02;
	bool is_interpolated;
	Eigen::Matrix<double, 3, 3> Rotdiff,Rot_w_1_p,Rot_w_1_m,Rot_w_2_p,Rot_w_2_m,Rot_w_3_p,Rot_w_3_m;
	Rotdiff(0,0) = 1.0; Rotdiff(0,1) = 0.0; Rotdiff(0,2) = 0.0;
	Rotdiff(1,0) = 0.0; Rotdiff(1,1) = 1.0; Rotdiff(1,2) = -w_h;
	Rotdiff(2,0) = 0.0; Rotdiff(2,1) = w_h; Rotdiff(2,2) = 1.0;
	
	Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
	double int_dist;
	for(int g = 0; g < 12; g++){
		A = Eigen::Matrix<double, 6, 6>::Zero();
		b = Eigen::Matrix<double, 6, 1>::Zero();
		Rot_w_1_p = Rotdiff*this->rot;
		Rotdiff(1,2) =  w_h;
		Rotdiff(2,1) = -w_h;
		Rot_w_1_m = Rotdiff*this->rot;
		Rotdiff(1,2) =  0;
		Rotdiff(2,1) =  0;
		Rotdiff(0,2) =  w_h;
		Rotdiff(2,0) = -w_h;
		Rot_w_2_p = Rotdiff*this->rot;
		Rotdiff(0,2) = -w_h;
		Rotdiff(2,0) =  w_h;
		Rot_w_2_m = Rotdiff*this->rot;
		Rotdiff(0,2) =  0;
		Rotdiff(2,0) =  0;
		Rotdiff(0,1) = -w_h;
		Rotdiff(1,0) =  w_h;
		Rot_w_3_p = Rotdiff*this->rot;
		Rotdiff(0,1) =  w_h;
		Rotdiff(1,0) = -w_h;
		Rot_w_3_m = Rotdiff*this->rot;
		for (i=0;i<point_cloud->width;i++){
			for (j=0;j<point_cloud->height;j++){
				point = point_cloud->at(i, j);
				if (isnan(point.x) || isnan(point.y) || isnan(point.z)){
					continue;
				}
				camera_point(0) = point.x;
				camera_point(1) = point.y;
				camera_point(2) = point.z;
				
				get_partial_derivative(sdf, camera_point, SDF_derivative,
				    Rot_w_1_p, Rot_w_1_m, Rot_w_2_p, Rot_w_2_m, Rot_w_3_p, Rot_w_3_m, w_h, is_interpolated, int_dist
				);
				if (!is_interpolated){
				    //cout  << "continue" << endl;
				    continue;
				}
				

				A = A + (SDF_derivative * SDF_derivative.transpose());
				b = b + int_dist * SDF_derivative;
				
			}
		}
		/*
		cout << "A"<<endl;
		cout << A << endl;
		cout << "b"<<endl;
		cout << b << endl;
		cout << "XI"<<endl;
		
		cout << SDF_derivative <<endl;
		cout << "MAX: " << SDF_derivative.maxCoeff()<<endl;
		cout << "TRans\n" << this->trans << endl;
		*/
		SDF_derivative = A.inverse()*b;
		
		this->trans(0) =this->trans(0)+ SDF_derivative(0,0);
		this->trans(1) =this->trans(1)+ SDF_derivative(1,0);
		this->trans(2) =this->trans(2)+ SDF_derivative(2,0);
		//cout << "TRans\n" << this->trans << endl;
		
		Rotdiff(0,0) = 1.0; 			Rotdiff(0,1) = -SDF_derivative(5,0);	Rotdiff(0,2) =  SDF_derivative(4,0);
		Rotdiff(1,0) = SDF_derivative(5,0);	Rotdiff(1,1) = 1.0; 			Rotdiff(1,2) = -SDF_derivative(3,0);
		Rotdiff(2,0) = -SDF_derivative(4,0); 	Rotdiff(2,1) = SDF_derivative(3,0); 	Rotdiff(2,2) = 1.0;
		//cout <<"r\n" << this-> rot << endl;
		this-> rot = Rotdiff * this->rot;
		this->set_camera_transformation(this->rot, this->trans);
		//cout <<"r\n" << this-> rot << endl;
		//reorthomolize
		/*cout <<"r1" << this-> rot.block(0,0,3,1) << endl;
		cout <<"f: "<< this-> rot.block(0,0,3,1).norm() << endl;
		cout <<"r1" << this-> rot.block(0,1,3,1) << endl;
		cout <<"f: "<< this-> rot.block(0,1,3,1).norm() << endl;
		cout <<"r1" << this-> rot.block(0,2,3,1) << endl;
		cout <<"f: "<< this-> rot.block(0,2,3,1).norm() << endl;*/
		
		
	}
	//cout << "----------------------------"<<endl;
	//cout << "----------------------------"<<endl;
	//cout << "----------------------------"<<endl;
}
//TODO precalculate 2*(sdf->m_div_width)
void CameraTracking::get_partial_derivative(SDF* sdf, Eigen::Vector3d& camera_point, Eigen::Matrix<double, 6, 1>& SDF_derivative,
	   Eigen::Matrix<double, 3, 3>& r1p,Eigen::Matrix<double, 3, 3>& r1m, 
	   Eigen::Matrix<double, 3, 3>& r2p,Eigen::Matrix<double, 3, 3>& r2m,
	   Eigen::Matrix<double, 3, 3>& r3p,Eigen::Matrix<double, 3, 3>& r3m,
	   double w_h,bool& is_interpolated, double& sdf_val){
	Vector3d current_world_point;
	Vector3d current_voxel_point;
	//we use central difference
	Vector3d before_world_point;
	Vector3d behind_world_point;
	Vector3d before_camera_point;
	Vector3d behind_camera_point;
	Vector3d before_voxel_point;
	Vector3d behind_voxel_point;
	float before;
	float behind;
	float delta_trans = 0.01;
	float delta_trans2 = 0.02;
	this->project_camera_to_world(camera_point, current_world_point);
	sdf->get_voxel_coordinates(current_world_point,current_voxel_point);
	if (current_voxel_point(0) < 0 || current_voxel_point(1) < 0 || current_voxel_point(2) < 0){
	  return ;
        }
        if (current_voxel_point(0) >= sdf->m || current_voxel_point(1) >= sdf->m || current_voxel_point(2) >= sdf->m){
	  return ;
        }
	sdf_val = sdf->interpolate_distance(current_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	//tx derivative
	this->trans(0) = this->trans(0) -delta_trans;
	before_world_point = this->rot*camera_point + this->trans;
	this->trans(0) = this->trans(0) +delta_trans2;
	behind_world_point = this->rot*camera_point + this->trans;
	this->trans(0) = this->trans(0) -delta_trans;
	sdf->get_voxel_coordinates(before_world_point,before_voxel_point);
	sdf->get_voxel_coordinates(behind_world_point,behind_voxel_point);
	before = sdf->interpolate_distance(before_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	behind = sdf->interpolate_distance(behind_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(0) = (before - behind)/delta_trans2;
	
	//ty derivative
	this->trans(1) = this->trans(1) -delta_trans;
	before_world_point = this->rot*camera_point + this->trans;
	this->trans(1) = this->trans(1) +delta_trans2;
	behind_world_point = this->rot*camera_point + this->trans;
	this->trans(1) = this->trans(1) -delta_trans;
	sdf->get_voxel_coordinates(before_world_point,before_voxel_point);
	sdf->get_voxel_coordinates(behind_world_point,behind_voxel_point);
	before = sdf->interpolate_distance(before_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	behind = sdf->interpolate_distance(behind_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(1) = (before - behind)/delta_trans2;
	
	//tz derivative 
	this->trans(2) = this->trans(2) -delta_trans;
	before_world_point = this->rot*camera_point + this->trans;
	this->trans(2) = this->trans(2) +delta_trans2;
	behind_world_point = this->rot*camera_point + this->trans;
	this->trans(2) = this->trans(2) -delta_trans;
	sdf->get_voxel_coordinates(before_world_point,before_voxel_point);
	sdf->get_voxel_coordinates(behind_world_point,behind_voxel_point);
	before = sdf->interpolate_distance(before_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	behind = sdf->interpolate_distance(behind_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(1) = (before - behind)/delta_trans2;
	
	//wx derivative
	before_world_point = r1m*camera_point + this->trans;
	behind_world_point = r1p*camera_point + this->trans;
	sdf->get_voxel_coordinates(before_world_point,before_voxel_point);
	sdf->get_voxel_coordinates(behind_world_point,behind_voxel_point);
	before = sdf->interpolate_distance(before_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	behind = sdf->interpolate_distance(behind_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(3) = (before - behind)/(2*(w_h));
	
	/*
	cout<<"..................................................\n";
	cout<<"before_world_point\n" << before_world_point <<endl;
	cout<<"behind_world_point\n" << behind_world_point <<endl;
	cout<<"before_voxel_point\n" << before_voxel_point <<endl;
	cout<<"behind_voxel_point\n" << behind_voxel_point <<endl;
	cout<<"before\n" << before <<endl;
	cout<<"behind\n" << behind <<endl;
	*/
	
	//wy derivative
	before_world_point = r2m*camera_point + this->trans;
	behind_world_point = r2p*camera_point + this->trans;
	sdf->get_voxel_coordinates(before_world_point,before_voxel_point);
	sdf->get_voxel_coordinates(behind_world_point,behind_voxel_point);
	before = sdf->interpolate_distance(before_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	behind = sdf->interpolate_distance(behind_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(4) = (before - behind)/(2*(w_h));
	
	/*
	cout<<"..................................................\n";
	cout<<"before_world_point\n" << before_world_point <<endl;
	cout<<"behind_world_point\n" << behind_world_point <<endl;
	cout<<"before_voxel_point\n" << before_voxel_point <<endl;
	cout<<"behind_voxel_point\n" << behind_voxel_point <<endl;
	cout<<"before\n" << before <<endl;
	cout<<"behind\n" << behind <<endl;
	*/
	
	//wz derivative
	before_world_point = r3m*camera_point + this->trans;
	behind_world_point = r3p*camera_point + this->trans;
	sdf->get_voxel_coordinates(before_world_point,before_voxel_point);
	sdf->get_voxel_coordinates(behind_world_point,behind_voxel_point);
	before = sdf->interpolate_distance(before_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	behind = sdf->interpolate_distance(behind_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(5) = (before - behind)/(2*(w_h));
	
	/*
	cout<<"..................................................\n";
	cout<<"before_world_point\n" << before_world_point <<endl;
	cout<<"behind_world_point\n" << behind_world_point <<endl;
	cout<<"before_voxel_point\n" << before_voxel_point <<endl;
	cout<<"behind_voxel_point\n" << behind_voxel_point <<endl;
	cout<<"before\n" << before <<endl;
	cout<<"behind\n" << behind <<endl;
	*/
}
