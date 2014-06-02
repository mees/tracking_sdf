#include "sdf_3d_reconstruction/sdf.h"

/*
 * Constructror destructor 
 */
SDF::SDF(int m, float width, float height, float depth): m(m), width(width),height(height), depth(depth){
	D = new float[this->m * this->m * this->m];
	W = new float[this->m * this->m * this->m];
	R = new float[this->m * this->m * this->m];
	G = new float[this->m * this->m * this->m];
	B = new float[this->m * this->m * this->m];
	numberOfVoxels = m * m * m;
	for (int i = 0; i<numberOfVoxels; i++) {
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

int SDF::getNumberOfVoxels() {
	return numberOfVoxels;
}

inline int SDF::get_array_index(Vector3i& voxel_coordinates){
	return this->m*this->m*voxel_coordinates(2)+this->m*voxel_coordinates(1)+voxel_coordinates(0);
}

inline void SDF::get_voxel_coordinates(int array_idx, Vector3i& voxel_coordinates){
	voxel_coordinates(0) = (int) array_idx%this->m;
	voxel_coordinates(1) = (int) (array_idx % (this->m*this->m))/this->m;
	voxel_coordinates(2) = (int) (array_idx/(this->m*this->m));
	std::cout << array_idx << "voxel_coordinates: " << voxel_coordinates << std::endl;
}
void SDF::get_global_coordinates(Vector3i& voxel_coordinates, Vector3d& global_coordinates){
  
		global_coordinates(0) = (this->width/((float)m)) * (voxel_coordinates(0)+0.5);
		global_coordinates(1) = (this->height/((float)m)) * (voxel_coordinates(1)+0.5);
		global_coordinates(2) = (this->depth/((float)m)) * (voxel_coordinates(2)+0.5);
		std::cout << "voxel_coordinates: " << voxel_coordinates << "global" << global_coordinates <<std::endl;
}
void SDF::create_circle(float radius, float center_x, float center_y,
		float center_z) {
	Vector3i voxel_coordinates;
	float x, y, z, d;
	for (int array_idx = 0; array_idx < numberOfVoxels; array_idx++){
		this -> get_voxel_coordinates(array_idx, voxel_coordinates);
		
		x = (this->width/((float)m)) * (voxel_coordinates(0)+0.5);
		y = (this->height/((float)m)) * (voxel_coordinates(1)+0.5);
		z = (this->depth/((float)m)) * (voxel_coordinates(2)+0.5);
		d = (x - center_x)*(x - center_x) + (y - center_y)*(y - center_y)+(z-center_z)*(z-center_z);
		D[array_idx] =  d-radius;
		std::cout << "global_coordinates: " << x <<", "<<y<<", "<<z<<" " << "distance" << d-radius <<std::endl;
		R[array_idx] = 1.0;
		G[array_idx] = x/width;
		B[array_idx] = y/width;
	}
}
void SDF::visualize(const std::string &file_name)
{
	pcl::PolygonMesh output;
	std::vector<pcl::Vertices> polygons;
	pcl::MarchingCubesSDF *mc;
	mc = new pcl::MarchingCubesSDF;
	mc->setIsoLevel (0.0f);
	mc->setGridResolution (this->m, this->m, this->m);
	mc->setGrid(this->D);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	mc->performReconstruction(output);	
	pcl::io::saveVTKFile(file_name, output);
	mc->performReconstruction (cloud, polygons);
	while (ros::ok())
	{
	  std::cout << "H"<<std::endl;

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "3dreconstruction";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
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
			std::cout << width << ", " << height << ", "<<depth<<std::endl;
			geometry_msgs::Point p1;
			p1.x = cloud.points[i*3].x/width;
			p1.y = cloud.points[i*3].y/height;
			p1.z = cloud.points[i*3].z/depth;
			std::cout << p1.x << ", " << p1.y << ", "<<p1.z<<std::endl;
	      
			geometry_msgs::Point p2;
			p2.x = cloud.points[i*3+1].x/width;
			p2.y = cloud.points[i*3+1].y/height;
			p2.z = cloud.points[i*3+1].z/depth;
			std::cout << p2.x << ", " << p2.y << ", "<<p2.z<<std::endl;
			
			geometry_msgs::Point p3;
			p3.x = cloud.points[i*3+2].x/width;
			p3.y = cloud.points[i*3+2].y/height;
			p3.z = cloud.points[i*3+2].z/depth;
			std::cout << p3.x << ", " << p3.y << ", "<<p3.z<<std::endl;
			
			marker.points.push_back(p1);
			marker.points.push_back(p2);
			marker.points.push_back(p3);
	      
			std_msgs::ColorRGBA c;
			c.r = 0.3;//x * 0.1;
			c.g = 1.0;//y * 0.1;
			c.b = 0.0;//z * 0.1;
			c.a = 1.0;
			marker.colors.push_back(c);
			marker.colors.push_back(c);
			marker.colors.push_back(c);
		  
		}
				
		// Publish the marker
		this->marker_publisher.publish(marker);

		
		r->sleep();
	}
}
