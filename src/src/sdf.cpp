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
	voxel_coordinates(0) = array_idx%this->m;
	voxel_coordinates(1) = (array_idx % (this->m*this->m))/this->m;
	voxel_coordinates(2) = (int) (array_idx/(this->m*this->m));
}
void SDF::get_global_coordinates(Vector3i& voxel_coordinates, Vector3d& global_coordinates){
  
		global_coordinates(0) = (this->width/((float)m)) * (voxel_coordinates(0)+0.5);
		global_coordinates(1) = (this->height/((float)m)) * (voxel_coordinates(1)+0.5);
		global_coordinates(2) = (this->depth/((float)m)) * (voxel_coordinates(2)+0.5);
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
	}
}
void SDF::visualize(const std::string &file_name)
{
	pcl::PolygonMesh output;
	pcl::MarchingCubesSDF *mc;
	mc = new pcl::MarchingCubesSDF;
	mc->setIsoLevel (0.0f);
	mc->setGridResolution (this->m, this->m, this->m);
	mc->setGrid(this->D);
	mc->performReconstruction (output);	
	pcl::io::saveVTKFile(file_name, output);
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
		
		geometry_msgs::Point p;
		p.x = 0.0;
		p.y = 0.0;
		p.z = 0.0;
      
		geometry_msgs::Point p2 = p;
		p2.x = p.x + 20.05;
      
		geometry_msgs::Point p3 = p;
		p3.x = p2.x;
		p3.z = p.z + 20.05;
      
		marker.points.push_back(p);
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
				
		// Publish the marker
		this->marker_publisher.publish(marker);


		visualization_msgs::Marker marker2;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker2.header.frame_id = "/map";
		marker2.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker2.  This serves to create a unique ID
		// Any marker2 sent with the same namespace and id will overwrite the old one
		marker2.ns = "3dreconstruction2";
		marker2.id = 0;

		// Set the marker2 type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker2.type = shape_type;

		// Set the marker2 action.  Options are ADD and DELETE
		marker2.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker2.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker2.pose.position.x = 0;
		marker2.pose.position.y = 0;
		marker2.pose.position.z = 0;
		marker2.pose.orientation.x = 0.0;
		marker2.pose.orientation.y = 0.0;
		marker2.pose.orientation.z = 0.0;
		marker2.pose.orientation.w = 1.0;

		// Set the scale of the marker2 -- 1x1x1 here means 1m on a side
		marker2.scale.x = 1.0;
		marker2.scale.y = 1.0;
		marker2.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker2.color.r = 0.0f;
		marker2.color.g = 1.0f;
		marker2.color.b = 0.0f;
		marker2.color.a = 1.0;

		marker2.lifetime = ros::Duration();


		// Publish the marker
		this->marker_publisher.publish(marker2);
		// Cycle between different shapes
		switch (shape_type)
		{
			case visualization_msgs::Marker::CUBE:
			  shape_type = visualization_msgs::Marker::SPHERE;
			  break;
			case visualization_msgs::Marker::SPHERE:
			  shape_type = visualization_msgs::Marker::ARROW;
			  break;
			case visualization_msgs::Marker::ARROW:
			  shape_type = visualization_msgs::Marker::CYLINDER;
			  break;
			case visualization_msgs::Marker::CYLINDER:
			  shape_type = visualization_msgs::Marker::CUBE;
			  break;
		}
		
		r->sleep();
	}
}
