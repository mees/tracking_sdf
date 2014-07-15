#include "sdf_3d_reconstruction/camera_tracking.h"
#include "sdf_3d_reconstruction/sdf.h"
CameraTracking::CameraTracking(int gauss_newton_max_iteration, float maximum_twist_diff){
      this->trans = Eigen::Vector3d(0,0,0);
      this->rot = Eigen::Matrix3d();
      this->rot << 1,0,0,\
		   0,0,1,\
		   0,1,0;
      this->rot_inv = Eigen::Matrix3d();
      this->rot_inv << 1,0,0,\
                       0,0,1,\
		       0,1,0;
      this->rot_inv_trans =  Eigen::Vector3d(0,0,0);
      //cout << this->rot;
      this->maximum_twist_diff = maximum_twist_diff;
      this->gauss_newton_max_iteration = gauss_newton_max_iteration;
  
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
	Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
	//twist_diff = (v1,v2,v3,w1,w2,w3)
	Eigen::Matrix<double, 6, 1> twist_diff = Eigen::Matrix<double, 6, 1>::Zero();
	double int_dist;
	bool stop = false;
	Vector3d r1,r2,r3;
	for(int g = 0; g < gauss_newton_max_iteration && !stop; g++){
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
		Rotdiff(0,0) = 1.0; Rotdiff(0,1) = 0.0; Rotdiff(0,2) = 0.0;
		Rotdiff(1,0) = 0.0; Rotdiff(1,1) = 1.0; Rotdiff(1,2) = -w_h;
		Rotdiff(2,0) = 0.0; Rotdiff(2,1) = w_h; Rotdiff(2,2) = 1.0;
		Rot_w_1_p = Rotdiff*this->rot;
		/* 
		 * R_w1- = 1.0 0.0 0.0
		 *         0.0 1.0  w1
		 *         0.0 -w1 1.0    
		 */
		Rotdiff(1,2) =  w_h;
		Rotdiff(2,1) = -w_h;
		Rot_w_1_m = Rotdiff*this->rot;
		/* 
		 * R_w2+ = 1.0 0.0  w2
		 *         0.0 1.0 0.0
		 *         -w2 0.0 1.0    
		 */
		Rotdiff(1,2) =  0;
		Rotdiff(2,1) =  0;
		Rotdiff(0,2) =  w_h;
		Rotdiff(2,0) = -w_h;
		Rot_w_2_p = Rotdiff*this->rot;
		/* 
		 * R_w2- = 1.0 0.0 -w2
		 *         0.0 1.0 0.0
		 *          w2 0.0 1.0    
		 */
		Rotdiff(0,2) = -w_h;
		Rotdiff(2,0) =  w_h;
		Rot_w_2_m = Rotdiff*this->rot;
		/* 
		 * R_w3+ = 1.0 -w3 0.0
		 *          w3 1.0 0.0
		 *         0.0 0.0 1.0    
		 */
		Rotdiff(0,2) =  0;
		Rotdiff(2,0) =  0;
		Rotdiff(0,1) = -w_h;
		Rotdiff(1,0) =  w_h;
		Rot_w_3_p = Rotdiff*this->rot;
		/* 
		 * R_w3- = 1.0  w3 0.0
		 *         -w3 1.0 0.0
		 *         0.0 0.0 1.0    
		 */
		Rotdiff(0,1) =  w_h;
		Rotdiff(1,0) = -w_h;
		Rot_w_3_m = Rotdiff*this->rot;
		//iterate all image points
		for (i=0;i<point_cloud->width;i++){
			for (j=0;j<point_cloud->height;j++){
				point = point_cloud->at(i, j);
				//does a good depth exist?
				if (isnan(point.x) || isnan(point.y) || isnan(point.z)){
					continue;
				}
				camera_point(0) = point.x;
				camera_point(1) = point.y;
				camera_point(2) = point.z;
				//get SDF_derivative
				get_partial_derivative(sdf, camera_point, SDF_derivative,
				    Rot_w_1_p, Rot_w_1_m, Rot_w_2_p, Rot_w_2_m, Rot_w_3_p, Rot_w_3_m, w_h, is_interpolated, int_dist
				);
				//we could calculate SDF_derivative at this point
				if (!is_interpolated){
				    continue;
				}
				
				A = A + (SDF_derivative * SDF_derivative.transpose());
				b = b + int_dist * SDF_derivative;
				
			}
		}
		//calculate our optimized gradient
		twist_diff = A.inverse()*b;
		/*cout << "A"<<endl;
		cout << A << endl;
		cout << "b"<<endl;
		cout << b << endl;
		cout << "twist_diff"<<endl;
		cout << twist_diff << endl;*/
		
		Rotdiff(0,0) = 1.0; 			Rotdiff(0,1) = twist_diff(5,0); 	Rotdiff(0,2) = -twist_diff(4,0);
		Rotdiff(1,0) = -twist_diff(5,0);	Rotdiff(1,1) = 1.0; 			Rotdiff(1,2) = twist_diff(3,0);
		Rotdiff(2,0) = twist_diff(4,0); 	Rotdiff(2,1) = -twist_diff(3,0); 	Rotdiff(2,2) = 1.0;
		//cout <<"r\n" << this-> rot << endl;
		Vector3d t_inv (twist_diff(0,0), twist_diff(1,0), twist_diff(2,0));
		t_inv = Rotdiff * t_inv;
		
		this->trans(0) =this->trans(0)- t_inv(0);
		this->trans(1) =this->trans(1)- t_inv(1);
		this->trans(2) =this->trans(2)- t_inv(2);
		//cout << "TRans\n" << this->trans << endl;
		this-> rot = Rotdiff * this->rot;
		//this-> rot = this->rot.householderQr().householderQ();
		Eigen::Quaterniond rot2;
		rot2 = (this->rot);
		cout << "Step:\n" <<this->trans << "\n"<<rot2.w() << " "<<rot2.x()<<" "<<rot2.y()<<" "<<rot2.z() <<endl;
		this->set_camera_transformation(this->rot, this->trans);
		
		if (twist_diff(0,0) < maximum_twist_diff && 
		    twist_diff(1,0) < maximum_twist_diff && 
		    twist_diff(2,0) < maximum_twist_diff && 
		    twist_diff(3,0) < maximum_twist_diff && 
		    twist_diff(4,0) < maximum_twist_diff && 
		    twist_diff(5,0) < maximum_twist_diff){
		    cout << "****** STOP *****" << endl;
		    stop = true;
		}
		  
		
		cout <<"r\n" << this-> rot << endl;
		//reorthomolize
		r1 = rot.block(0,0,3,1);
		r2 = rot.block(0,1,3,1);
		r3 = rot.block(0,2,3,1);
		r1 = r1/r1.norm();
		r2 = (r2-(r1.dot(r2))*r1)/r2.norm();
		r3 = (r3-(r1.dot(r3))*r1 -(r2.dot(r3))*r2)/r3.norm();
		
		rot.block(0,0,3,1) = r1;
		rot.block(0,1,3,1) = r2;
		rot.block(0,2,3,1) = r3;
		cout <<"r\n" << this-> rot << endl;
		/*cout <<"r1" << this-> rot.block(0,0,3,1) << endl;
		cout <<"f: "<< this-> rot.block(0,0,3,1).norm() << endl;
		cout <<"r1" << this-> rot.block(0,1,3,1) << endl;
		cout <<"f: "<< this-> rot.block(0,1,3,1).norm() << endl;
		cout <<"r1" << this-> rot.block(0,2,3,1) << endl;
		cout <<"f: "<< this-> rot.block(0,2,3,1).norm() << endl;*/
		
		
	}
	cout << "----------------------------"<<endl;
	cout << "----------------------------"<<endl;
	cout << "----------------------------"<<endl;
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
	Vector3d plus_h_world_point;
	Vector3d minus_h_world_point;
	Vector3d plus_h_camera_point;
	Vector3d minus_h_camera_point;
	Vector3d plus_h_voxel_point;
	Vector3d minus_voxel_point;
	float plus_h_sdf_value;
	float minus_h_sdf_value;
	float delta_trans = 0.01;
	float delta_trans2 = 0.02;
	this->project_camera_to_world(camera_point, current_world_point);
	sdf->get_voxel_coordinates(current_world_point,current_voxel_point);
	if (current_voxel_point(0) < 0 || current_voxel_point(1) < 0 || current_voxel_point(2) < 0){
	  //cout << "point not in voxel grid";
	  return ;
        }
        if (current_voxel_point(0) >= sdf->m || current_voxel_point(1) >= sdf->m || current_voxel_point(2) >= sdf->m){
	  return ;
        }
	sdf_val = sdf->interpolate_distance(current_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	//tx derivative
	this->trans(0) = this->trans(0) +delta_trans;
	plus_h_world_point = this->rot*camera_point + this->trans;
	this->trans(0) = this->trans(0) -delta_trans2;
	minus_h_world_point = this->rot*camera_point + this->trans;
	this->trans(0) = this->trans(0) +delta_trans;
	sdf->get_voxel_coordinates(plus_h_world_point,plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point,minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(0) = (plus_h_sdf_value - minus_h_sdf_value)/delta_trans2;
	
	//ty derivative
	this->trans(1) = this->trans(1) +delta_trans;
	plus_h_world_point = this->rot*camera_point + this->trans;
	this->trans(1) = this->trans(1) -delta_trans2;
	minus_h_world_point = this->rot*camera_point + this->trans;
	this->trans(1) = this->trans(1) +delta_trans;
	sdf->get_voxel_coordinates(plus_h_world_point,plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point,minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(1) = (plus_h_sdf_value - minus_h_sdf_value)/delta_trans2;
	
	//tz derivative 
	this->trans(2) = this->trans(2) +delta_trans;
	plus_h_world_point = this->rot*camera_point + this->trans;
	this->trans(2) = this->trans(2) -delta_trans2;
	minus_h_world_point = this->rot*camera_point + this->trans;
	this->trans(2) = this->trans(2) +delta_trans;
	sdf->get_voxel_coordinates(plus_h_world_point,plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point,minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(2) = (plus_h_sdf_value - minus_h_sdf_value)/delta_trans2;
	
	//wx derivative
	plus_h_world_point = r1p*camera_point + this->trans;
	minus_h_world_point = r1m*camera_point + this->trans;
	sdf->get_voxel_coordinates(plus_h_world_point,plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point,minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(3) = (plus_h_sdf_value - minus_h_sdf_value)/(2*(w_h));
	
	/*
	cout<<"..................................................\n";
	cout<<"plus_h_world_point\n" << plus_h_world_point <<endl;
	cout<<"minus_h_world_point\n" << minus_h_world_point <<endl;
	cout<<"plus_h_voxel_point\n" << plus_h_voxel_point <<endl;
	cout<<"minus_voxel_point\n" << minus_voxel_point <<endl;
	cout<<"plus_h_sdf_value\n" << plus_h_sdf_value <<endl;
	cout<<"minus_h_sdf_value\n" << minus_h_sdf_value <<endl;
	*/
	
	//wy derivative
	plus_h_world_point = r2p*camera_point + this->trans;
	minus_h_world_point = r2m*camera_point + this->trans;
	sdf->get_voxel_coordinates(plus_h_world_point,plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point,minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(4) = (plus_h_sdf_value - minus_h_sdf_value)/(2*(w_h));
	
	/*
	cout<<"..................................................\n";
	cout<<"plus_h_world_point\n" << plus_h_world_point <<endl;
	cout<<"minus_h_world_point\n" << minus_h_world_point <<endl;
	cout<<"plus_h_voxel_point\n" << plus_h_voxel_point <<endl;
	cout<<"minus_voxel_point\n" << minus_voxel_point <<endl;
	cout<<"plus_h_sdf_value\n" << plus_h_sdf_value <<endl;
	cout<<"minus_h_sdf_value\n" << minus_h_sdf_value <<endl;
	*/
	
	//wz derivative
	plus_h_world_point = r3p*camera_point + this->trans;
	minus_h_world_point = r3m*camera_point + this->trans;
	sdf->get_voxel_coordinates(plus_h_world_point,plus_h_voxel_point);
	sdf->get_voxel_coordinates(minus_h_world_point,minus_voxel_point);
	plus_h_sdf_value = sdf->interpolate_distance(plus_h_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	minus_h_sdf_value = sdf->interpolate_distance(minus_voxel_point, is_interpolated);
	if (!is_interpolated) return;
	SDF_derivative(5) = (plus_h_sdf_value - minus_h_sdf_value)/(2*(w_h));
	
	/*
	cout<<"..................................................\n";
	cout<<"plus_h_world_point\n" << plus_h_world_point <<endl;
	cout<<"minus_h_world_point\n" << minus_h_world_point <<endl;
	cout<<"plus_h_voxel_point\n" << plus_h_voxel_point <<endl;
	cout<<"minus_voxel_point\n" << minus_voxel_point <<endl;
	cout<<"plus_h_sdf_value\n" << plus_h_sdf_value <<endl;
	cout<<"minus_h_sdf_value\n" << minus_h_sdf_value <<endl;
	*/
}
