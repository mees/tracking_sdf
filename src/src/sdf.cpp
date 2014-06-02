#include "sdf_3d_reconstruction/sdf.h"

/*
 * Constructror destructor 
 */
SDF::SDF(int m, float width, float height, float depth): m(m), width(width),height(height), depth(depth){
	D = new float[this->m * this->m * this->m];
	W = new float[this->m * this->m * this->m];
	numberOfVoxels = m * m * m;
	for (int i = 0; i<numberOfVoxels; i++) {
		D[i] = 0;
		W[i] = 0;
	}
}
SDF::~SDF(){
  
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
}
