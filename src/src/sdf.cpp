#include "sdf_3d_reconstruction/sdf.h"


using namespace Eigen;
/*
 * Constructror destructor 
 */
SDF::SDF(int m, float width, float height, float depth,Vector3d& sdf_origin, float distance_delta, float distance_epsilon): m(m), width(width),height(height), depth(depth),sdf_origin(sdf_origin), distance_delta(distance_delta), distance_epsilon(distance_epsilon){
	number_of_voxels = m * m * m;
	D = new float[number_of_voxels];
	global_coords = new Vector3d[number_of_voxels];
	voxel_coords = new Vector3i[(m-2)*(m-2)*(m-2)];
	W = new float[number_of_voxels];
	Color_W = new float[number_of_voxels];
	R = new float[number_of_voxels];
	G = new float[number_of_voxels];
	B = new float[number_of_voxels];
	m_squared = m*m;
	m_div_height = m/height;
	m_div_width = m/width;
	m_div_depth = m/depth;
	initial_update_done = false;
	finish_visualization_thread = false;

	Vector3i voxel_coordinates;
	Vector3d global_coordinates;
	int vox_ind = 0;
	for (int i = 0; i<number_of_voxels; i++) {
		D[i] = width+height+depth;
		Color_W[i] = 0;
		W[i] = 0;
		R[i] = 0.4;
		G[i] = 0.4;
		B[i] = 0.4;
		get_voxel_coordinates(i, voxel_coordinates);
		if((voxel_coordinates(0)>0 && voxel_coordinates(0)<m-1) && (voxel_coordinates(1)>0 && voxel_coordinates(1)<m-1) && (voxel_coordinates(2)>0 && voxel_coordinates(2)<m-1)){
		voxel_coords[vox_ind] = voxel_coordinates;
		vox_ind++;
		}
		get_global_coordinates(voxel_coordinates, global_coordinates);
		global_coords[i] = global_coordinates;
	}
	mc = new pcl::MarchingCubesSDF;
	mc->setIsoLevel (0.0f);
	mc->setGridResolution (this->m, this->m, this->m);
	mc->setBBox(this->width, this->height, this->depth);
	mc->setGrid(this->D);
	mc->setW(this->W);
	mc->setVoxelCoordinates(this->voxel_coords);
	this->register_visualization();
}
SDF::~SDF(){
  
}
void SDF::register_visualization(){
	ros::NodeHandle n;
	marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	shape_type = visualization_msgs::Marker::TRIANGLE_LIST;
	shape_type = visualization_msgs::Marker::CUBE;
}

void SDF::create_cuboid(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z){
	Vector3i voxel_coordinates;
	Vector3d global_coordinates;
	double x, y, z;
	float dx, dy, dz;
	for (int array_idx = 0; array_idx < this->number_of_voxels; array_idx++){
		this -> get_voxel_coordinates(array_idx, voxel_coordinates);
		this-> get_global_coordinates(voxel_coordinates, global_coordinates);
		x = global_coordinates(0);
		y = global_coordinates(1);
		z = global_coordinates(2);
		dx = fmin(fabs(x-min_x),fabs(x-max_x));
		dy = fmin(fabs(y-min_y),fabs(y-max_y));
		dz = fmin(fabs(z-min_z),fabs(z-max_z));
		if (x < max_x && x > min_x && y < max_y && y > min_y && z < max_z && z > min_z ){
			D[array_idx] =  -fmin(dx,fmin(dy,dz));    
		}
		else{
			D[array_idx] =  fmin(dx,fmin(dy,dz));    
		}
		W[array_idx] = 0.001;
		R[array_idx] = 1.0;
		G[array_idx] = 0.0;
		B[array_idx] = 0.0;
		if (dz < 0.017 && dz > -0.017){
		  B[array_idx] = 1.0;
		  W[array_idx] = 1.0;
		  R[array_idx] = 0.0;
		}
		if ((dz > 0.017 && dz < 0.034) || (dz < -0.017 && dz > -0.034)){
		  B[array_idx] = 0.0;
		  W[array_idx] = 0.01;
		  R[array_idx] = 1.0;
		  G[array_idx] = 1.0;
		}
	}
}
void SDF::create_circle(float radius, float center_x, float center_y,
		float center_z) {
	Vector3i voxel_coordinates;
	Vector3d global_coordinates;
	double x, y, z, d, b;
	for (int array_idx = 0; array_idx < this->number_of_voxels; array_idx++){
		this -> get_voxel_coordinates(array_idx, voxel_coordinates);
		this-> get_global_coordinates(voxel_coordinates, global_coordinates);
		x = global_coordinates(0);
		y = global_coordinates(1);
		z = global_coordinates(2);
		d = sqrt((x - center_x)*(x - center_x) + (y - center_y)*(y - center_y)+(z-center_z)*(z-center_z));
		
		D[array_idx] =  d-radius;
		W[array_idx] = 1.0;
		R[array_idx] = 0.0;
		G[array_idx] = 0.0;
		
		B[array_idx] = (x/this->width);
		//std::cout << "Blue: "<<(x/this->width) <<" "<< x << std::endl;
		if (B[array_idx] > 1){
		  B[array_idx] = 1.0;
		}
		if (B[array_idx] < 0.0){
		  B[array_idx] = 0.0;
		}
	}
}
float SDF::interpolate_distance(const Vector3d& voxel_coordinates, bool& is_interpolated) const{
	//Vector3d voxel_coordinates;
	//get_voxel_coordinates(world_coordinates, voxel_coordinates);
	float i = voxel_coordinates(0);
	float j = voxel_coordinates(1);
	float k = voxel_coordinates(2);
	float w_sum = 0.0;
	float sum_d = 0.0;
	Vector3i current_voxel;
	float w = 0;
	float volume;
	int a_idx;
	is_interpolated = false;
	for (int i_offset = 0; i_offset < 2; i_offset++){
	  for (int j_offset = 0; j_offset < 2; j_offset++){
	    for (int k_offset = 0; k_offset < 2; k_offset++){
	      current_voxel(0) = ((int) i)+i_offset;
	      current_voxel(1) = ((int) j)+j_offset;
	      current_voxel(2) = ((int) k)+k_offset;
	      volume = fabs(current_voxel(0)-i) + fabs(current_voxel(1)-j)+ fabs(current_voxel(2)-k);
	      a_idx = get_array_index(current_voxel);
	      if (a_idx != -1){
		  if (W[a_idx] >0){
			is_interpolated = true;
			if (volume < 0.00001){
				return this->D[a_idx];
			}
			w = 1.0/volume;
			w_sum += w;
			sum_d +=  w*this->D[a_idx];
		  }
	      }
	    }
	  }
	}
	return sum_d / w_sum;
}
void SDF::interpolate_color(geometry_msgs::Point& global_coords, std_msgs::ColorRGBA& color) const{
	Vector3d global_coordinates;
	Vector3d voxel_coordinates;
	global_coordinates(0) = global_coords.x;
	global_coordinates(1) = global_coords.y;
	global_coordinates(2) = global_coords.z;
	get_voxel_coordinates(global_coordinates, voxel_coordinates);

	float i = voxel_coordinates(0);
	float j = voxel_coordinates(1);
	float k = voxel_coordinates(2);
	float w_sum = 0.0;
	float aux = 0;
	Vector3d sum_c;
	color.r = 0.0;
	color.g = 0.0;
	color.b = 0.0;
	color.a = 1.0;
	Vector3i current_voxel;
	float w = 0;
	float volume;
	int a_idx;
	for (int i_offset = 0; i_offset < 2; i_offset++){
	  for (int j_offset = 0; j_offset < 2; j_offset++){
	    for (int k_offset = 0; k_offset < 2; k_offset++){
	      current_voxel(0) = ((int) i)+i_offset;
	      current_voxel(1) = ((int) j)+j_offset;
	      current_voxel(2) = ((int) k)+k_offset;
	      volume = fabs(current_voxel(0)-i) + fabs(current_voxel(1)-j)+ fabs(current_voxel(2)-k);
	      a_idx = get_array_index(current_voxel);
	      if (a_idx != -1){
		  if (Color_W[a_idx] >0){
		    if (volume < 0.00001){
		      color.r =  this->R[a_idx];
		      color.g =  this->G[a_idx];
		      color.b =  this->B[a_idx];
		      return;
		    }
		    
		    w = 1.0/volume;
		    w_sum += w;
		    color.r +=  w*this->R[a_idx];
		    color.g +=  w*this->G[a_idx];
		    color.b +=  w*this->B[a_idx];
		  }
	      }
	    }
	  }
	}
	aux = w_sum*255.0;
	color.r /= aux;
	color.g /= aux;
	color.b /= aux;
}






void SDF::update(CameraTracking* camera_tracking, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr normals){
		ros::Time t0 = ros::Time::now();
		static int count_id = 0;
    if (!camera_tracking->isKFilled) {
	    cout << "Camera Matrix not received. Start rosbag file!" << endl;
	    exit(0);
    } else {

#pragma omp parallel for
	for (int idx=0;idx<number_of_voxels;idx++){
		Vector3i voxel_coordinates;
		Vector3d cam_vect(0,0,1);
		Vector3d global_coordinates, camera_point, camera_point_img,normal_eigen;
		Vector2d image_point;
		float d_new, scalar = 0;
		float w_new, w_old;
		int i_image, j_image;
		pcl::PointXYZRGB point;
		pcl::Normal normal;

		global_coordinates = global_coords[idx];
		camera_tracking->project_world_to_camera(global_coordinates, camera_point);
		//if voxel behind the camera
		if (camera_point(2) < 0){
			continue;
		}
		camera_tracking->project_camera_to_image_plane(camera_point, image_point);
		i_image = image_point(0);
		j_image = image_point(1);
		//if voxel not inside the image
		if (i_image >= cloud_filtered->width || j_image >= cloud_filtered->height || i_image< 0 || j_image < 0){
			continue;
		}
		point = cloud_filtered->at(i_image, j_image);
		normal = normals->at(i_image, j_image);
		//normal could not be computed
		if (isnan(point.x) || isnan(point.y) || isnan(normal.normal_x) || isnan(normal.normal_y) || isnan(normal.normal_z)){
			continue;
		}
		camera_point_img(0) =  point.x;
		camera_point_img(1) =  point.y;
		camera_point_img(2) =  point.z;
		double pointToPointDistance, pointToPlaneDistance = 0;
		//projectivePointToPointDistance(camera_point(2), camera_point_img(2), pointToPointDistance);

		normal_eigen(0) = normal.normal_x;
		normal_eigen(1) = normal.normal_y;
		normal_eigen(2) = normal.normal_z;
		projectivePointToPlaneDistance(camera_point, camera_point_img, normal_eigen, pointToPlaneDistance);
		//cout<<"camera_point: \n"<<camera_point<<" camera_point_img: \n"<<camera_point_img<<" point2point dist: \n"<<pointToPointDistance<<" poin2plane: \n"<<pointToPlaneDistance<<endl;
		d_new = pointToPlaneDistance;
		//exponential weighting function Eq. 31
		w_new = 1.0;
		if (d_new >= this->distance_epsilon && d_new <= this->distance_delta){
		  w_new = exp(-0.5*(d_new - this->distance_epsilon)*(d_new - this->distance_epsilon));
		}
		if (d_new > distance_delta){
		  w_new = 0.0;
		  continue;
		}
		//truncation function Eq. 28
		if (d_new < -distance_delta){
		  d_new = -distance_delta;
		}

		w_old = W[idx];
		W[idx] = w_old + w_new;
		
		D[idx] = (w_old * D[idx] + w_new * d_new)/W[idx];
		
		double cosine = fabs((cam_vect.dot(normal_eigen)))/normal_eigen.norm();
		//std::cout <<"cosine:  "<< cosine<<std::endl;;

		
		w_old = Color_W[idx];
		w_new = w_new * cosine;
		Color_W[idx] = w_old + w_new;
		
		R[idx] = (w_old * R[idx] + w_new * point.r)/Color_W[idx];
		G[idx] = (w_old * G[idx] + w_new * point.g)/Color_W[idx];
		B[idx] = (w_old * B[idx] + w_new * point.b)/Color_W[idx];
	}
	std::cout << "update method: "<<(ros::Time::now()-t0).toSec()<< std::endl;
	if(count_id==0){
	std::lock_guard<std::mutex> lk(cv_m);
	initial_update_done = true;
	cv.notify_one();
	}
	count_id++;
	//this->visualize();
	}
}

void SDF::visualize(double frequency)
{
	ros::Rate r(frequency);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	std::unique_lock<std::mutex> lk(cv_m);
	//std::cerr << "Waiting... \n";
	 cv.wait(lk, [this](){return initial_update_done == true;});
	while (ros::ok() && !finish_visualization_thread)
	{
		ros::Time t0 = ros::Time::now();
		mc->performReconstruction (cloud);
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time();
		marker.ns = "3dreconstruction";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points.reserve(cloud.size());
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.color.r = 0.0;
		marker.color.a = 1.0;
		for(int i = 0; i < cloud.size()/3; i++){
			//std::cout << width << ", " << height << ", "<<depth<<": "<<std::endl;
			//std::cout <<cloud.points[i*3].x<<", "<<cloud.points[i*3].y<<", "<<cloud.points[i*3].z<<std::endl;
			geometry_msgs::Point p1;
			p1.x = cloud.points[i*3].x+this->sdf_origin(0);
			p1.y = cloud.points[i*3].y+this->sdf_origin(1);
			p1.z = cloud.points[i*3].z+this->sdf_origin(2);
			//std::cout << p1.x << ", " << p1.y << ", "<<p1.z<<std::endl;
	      
			geometry_msgs::Point p2;
			p2.x = cloud.points[i*3+1].x+this->sdf_origin(0);
			p2.y = cloud.points[i*3+1].y+this->sdf_origin(1);
			p2.z = cloud.points[i*3+1].z+this->sdf_origin(2);
			//std::cout << p2.x << ", " << p2.y << ", "<<p2.z<<std::endl;
			
			geometry_msgs::Point p3;
			p3.x = cloud.points[i*3+2].x+this->sdf_origin(0);
			p3.y = cloud.points[i*3+2].y+this->sdf_origin(1);
			p3.z = cloud.points[i*3+2].z+this->sdf_origin(2);
			//std::cout << p3.x << ", " << p3.y << ", "<<p3.z<<std::endl;
			
			marker.points.push_back(p1);
			marker.points.push_back(p2);
			marker.points.push_back(p3);
	      
			std_msgs::ColorRGBA c;
			this->interpolate_color(p1,c);
			marker.colors.push_back(c);
			this->interpolate_color(p2,c);
			marker.colors.push_back(c);
			this->interpolate_color(p3,c);
			marker.colors.push_back(c);
		  
		}
		// Publish the marker
		this->marker_publisher.publish(marker);
		std::cout << "visualize method: "<<(ros::Time::now()-t0).toSec()<< std::endl;
		r.sleep();
	}

}
