#include "sdf_3d_reconstruction/sdf.h"

using namespace Eigen;
/*
 * Constructror destructor 
 */
SDF::SDF(int m, float width, float height, float depth): m(m), width(width),height(height), depth(depth){
	D = new float[this->m * this->m * this->m];
	W = new float[this->m * this->m * this->m];
	R = new float[this->m * this->m * this->m];
	G = new float[this->m * this->m * this->m];
	B = new float[this->m * this->m * this->m];
	number_of_voxels = m * m * m;
	for (int i = 0; i<number_of_voxels; i++) {
		D[i] = 0;
		W[i] = 0;
		R[i] = 0;
		G[i] = 0;
		B[i] = 0;
	}
	this->register_visualization();
}
SDF::~SDF(){
  
}
void SDF::register_visualization(){
	ros::NodeHandle n;
	r = new ros::Rate(1);
	marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	shape_type = visualization_msgs::Marker::TRIANGLE_LIST;
	shape_type = visualization_msgs::Marker::CUBE;
}

int SDF::get_number_of_voxels() {
	return number_of_voxels;
}

inline int SDF::get_array_index(Vector3i& voxel_coordinates){
	int _idx = this->m*this->m*voxel_coordinates(2)+this->m*voxel_coordinates(1)+voxel_coordinates(0);
	if (_idx < 0 || _idx >= this->m*this->m*this->m){
	  std::cout << "ooo"<<std::endl;
	}
	  
	return _idx;
}

inline void SDF::get_voxel_coordinates(int array_idx, Vector3i& voxel_coordinates){
	voxel_coordinates(0) = (int) array_idx%this->m;
	voxel_coordinates(1) = (int) (array_idx % (this->m*this->m))/this->m;
	voxel_coordinates(2) = (int) (array_idx/(this->m*this->m));
}
void SDF::get_global_coordinates(Vector3i& voxel_coordinates, Vector3d& global_coordinates){
  
	global_coordinates(0) = (this->width/((float)m)) * (voxel_coordinates(0)+0.5);
	global_coordinates(1) = (this->height/((float)m)) * (voxel_coordinates(1)+0.5);
	global_coordinates(2) = (this->depth/((float)m)) * (voxel_coordinates(2)+0.5);
}
void SDF::get_voxel_coordinates(Vector3d& global_coordinates, Vector3i& voxel_coordinates){
	voxel_coordinates(0) = (int)((global_coordinates(0)/this->width)*m -0.5);
	voxel_coordinates(1) = (int)((global_coordinates(1)/this->height)*m -0.5);
	voxel_coordinates(2) = (int)((global_coordinates(2)/this->depth)*m -0.5);
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

float SDF::interpolate_distance(Vector3d& world_coordinates){
	float i = ((world_coordinates(0)/this->width)*m -0.5);
	float j = ((world_coordinates(1)/this->height)*m -0.5);
	float k = ((world_coordinates(2)/this->depth)*m -0.5);
	float w_sum = 0.0;
	float sum_d = 0.0;
	Vector3i current_voxel;
	for (int i_offset = 0; i_offset < 2; i_offset++){
	  for (int j_offset = 0; j_offset < 2; j_offset++){
	    for (int k_offset = 0; k_offset < 2; k_offset++){
	      current_voxel(0) = ((int) i)+i_offset;
	      current_voxel(1) = ((int) j)+j_offset;
	      current_voxel(2) = ((int) k)+k_offset;
	      float dist = (current_voxel(0)-i)*(current_voxel(0)-i) + (current_voxel(1)-j)*(current_voxel(1)-j)+ (current_voxel(2)-k)*(current_voxel(2)-k);
	      int a_idx = get_array_index(current_voxel);
	      if (dist < 0.001){
		return this->D[a_idx];
	      }
	      float w = 1.0/dist;
	      w_sum += w;
	      sum_d +=  w*this->D[a_idx];
	    }
	  }
	}
	return sum_d / w_sum;
}
void SDF::interpolate_color(pcl::PointXYZ& global_coords, std_msgs::ColorRGBA& color){
	float i = ((global_coords.x/this->width)*m -0.5);
	float j = ((global_coords.y/this->height)*m -0.5);
	float k = ((global_coords.z/this->depth)*m -0.5);
	float w_sum = 0.0;
	Vector3d sum_c;
	color.r = 0.0;
	color.g = 0.0;
	color.b = 0.0;
	color.a = 1.0;
	Vector3i current_voxel;
	for (int i_offset = 0; i_offset < 2; i_offset++){
	  for (int j_offset = 0; j_offset < 2; j_offset++){
	    for (int k_offset = 0; k_offset < 2; k_offset++){
	      current_voxel(0) = ((int) i)+i_offset;
	      current_voxel(1) = ((int) j)+j_offset;
	      current_voxel(2) = ((int) k)+k_offset;
	      float dist = (current_voxel(0)-i)*(current_voxel(0)-i) + (current_voxel(1)-j)*(current_voxel(1)-j)+ (current_voxel(2)-k)*(current_voxel(2)-k);
	      int a_idx = get_array_index(current_voxel);
	      if (dist < 0.001){
		color.r =  this->R[a_idx];
		color.g =  this->G[a_idx];
		color.b =  this->B[a_idx];
		return;
	      }
	      float w = 1.0/dist;
	      w_sum += w;
	      color.r +=  w*this->R[a_idx];
	      color.g +=  w*this->G[a_idx];
	      color.b +=  w*this->B[a_idx];
	    }
	  }
	}
	color.r /= w_sum;
	color.g /= w_sum;
	color.b /= w_sum;
}

void SDF::update(CameraTracking* camera_tracking, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr normals){
		ros::Time t0 = ros::Time::now();
    
    if (!camera_tracking->isKFilled) {
	    cout << "Camera Matrix not received. Start rosbag file!" << endl;
	    exit(0);
    } else {
		//std::cout<< d_w << "," << d_h << std::endl;
		/*for (int idx = 0; idx < cloud_filtered->size(); ++idx) {
			int i = idx%cloud_filtered->height;
			int j = int(idx/cloud_filtered->height);
			float z = cloud_filtered->points[idx].z;
			Vector3d  camera_point;
			camera_point(0) =  cloud_filtered->points[idx].x;
			camera_point(1) =  cloud_filtered->points[idx].y;
			camera_point(2) =  cloud_filtered->points[idx].z;
			Vector3d a = camera_tracking->K * camera_point;
			cout << "--------------------------------------------_"<<endl;
			cout << cloud_filtered->points[i] << endl;
			cout << camera_point <<endl ;
			cout <<a <<endl;
			cout << i<< ", "<<j <<endl;
			Vector2d image_point;
			camera_tracking->project_camera_to_image_plane(camera_point, image_point);    
		}*/
		for (int idx = 0; idx < this->get_number_of_voxels(); idx++) {
			Vector3i voxel_coordinates;
			Vector3d global_coordinates, camera_point;
			Vector2d image_point;
			this->get_voxel_coordinates(idx,voxel_coordinates);
			this->get_global_coordinates(voxel_coordinates, global_coordinates);
			camera_tracking->project_world_to_camera(global_coordinates, camera_point);
			camera_tracking->project_camera_to_image_plane(camera_point, image_point);
			//cout<< global_coordinates<<endl;
			//cout << camera_point<<endl;
			//cout <<image_point <<endl;
			//point to point
			float z_voxel = camera_point(2);
			int i = image_point(0);
			int j = image_point(1);
			if (i < cloud_filtered->width && j < cloud_filtered->height && i> 0 && j > 0){
				int cloud_idx = j*cloud_filtered->height + i;
				if (!isnan(cloud_filtered->points[cloud_idx].x) && !isnan(cloud_filtered->points[cloud_idx].y)){
					//cout << i<< ", "<<j <<", " << cloud_idx <<endl;
					float z_img =  cloud_filtered->points[cloud_idx].z;
					//cout << z_img << " " << z_voxel<<endl;
 					//print point-to-point
					//cout <<z_voxel-z_img<<endl;
					float w_old = W[idx];
					float d_old = D[idx];
					W[idx] = w_old + 1.0;
					D[idx] = w_old/W[idx] * d_old + 1.0/W[idx] * (z_voxel-z_img);
				}
			}
		}
		std::cout << (ros::Time::now()-t0).toSec()<< std::endl;
		this->visualize();
	}
}

void SDF::visualize()
{
	ros::Time t0 = ros::Time::now();
	pcl::PolygonMesh output;
	std::vector<pcl::Vertices> polygons;
	pcl::MarchingCubesSDF *mc;
	mc = new pcl::MarchingCubesSDF;
	mc->setIsoLevel (0.0f);
	mc->setGridResolution (this->m, this->m, this->m);
	mc->setBBox(this->width, this->height, this->depth);
	mc->setGrid(this->D);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	mc->performReconstruction (cloud, polygons);
	if (ros::ok())
	{
		//std::cout << "H"<<std::endl;

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
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
			p1.x = cloud.points[i*3].x/width;
			p1.y = cloud.points[i*3].y/height;
			p1.z = cloud.points[i*3].z/depth;
			//std::cout << p1.x << ", " << p1.y << ", "<<p1.z<<std::endl;
	      
			geometry_msgs::Point p2;
			p2.x = cloud.points[i*3+1].x/width;
			p2.y = cloud.points[i*3+1].y/height;
			p2.z = cloud.points[i*3+1].z/depth;
			//std::cout << p2.x << ", " << p2.y << ", "<<p2.z<<std::endl;
			
			geometry_msgs::Point p3;
			p3.x = cloud.points[i*3+2].x/width;
			p3.y = cloud.points[i*3+2].y/height;
			p3.z = cloud.points[i*3+2].z/depth;
			//std::cout << p3.x << ", " << p3.y << ", "<<p3.z<<std::endl;
			
			marker.points.push_back(p1);
			marker.points.push_back(p2);
			marker.points.push_back(p3);
	      
			std_msgs::ColorRGBA c;
			this->interpolate_color(cloud.points[i*3],c);
			marker.colors.push_back(c);
			this->interpolate_color(cloud.points[i*3+1],c);
			marker.colors.push_back(c);
			this->interpolate_color(cloud.points[i*3+2],c);
			marker.colors.push_back(c);
		  
		}
		// Publish the marker
		this->marker_publisher.publish(marker);
		//r->sleep();
	}
	std::cout << (ros::Time::now()-t0).toSec()<< std::endl;
}
