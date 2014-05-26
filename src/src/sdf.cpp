#include "MarchingCubesSDF.h"
#include "sdf.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
SDF::SDF(int m, float width, float height, float depth): m(m), width(width),height(height), depth(depth){
	D = new float[this->m * this->m * this->m];
	W = new float[this->m * this->m * this->m];
}
SDF::~SDF(){
  
}
inline int SDF::get_array_index(grid_index& gi){
	return this->m*this->m*gi.k+this->m*gi.j+gi.i;
}
inline void SDF::get_grid_index(int array_idx, grid_index& gi){
	gi.i = array_idx%this->m;
	gi.j = (array_idx % (this->m*this->m))/this->m;
	gi.k = (int) (array_idx/(this->m*this->m));
}
void SDF::create_circle(float radius, float center_x, float center_y,
		float center_z) {

	grid_index gi;
	float x, y, z, d;
	for (int array_idx = 0; array_idx < this->m*this->m*this->m; array_idx++){
		this -> get_grid_index(array_idx, gi);
		x = (this->width/((float)m)) * gi.i;
		y = (this->height/((float)m)) * gi.j;
		z = (this->depth/((float)m)) * gi.k;
		
		d = (x - center_x)*(x - center_x) + (y - center_y)*(y - center_y)+(z-center_z)*(z-center_z);
		//std::cout << radius << " - " << d << " = " << radius -d << std::endl;
		D[array_idx] = radius - d;
		
	}
}
void SDF::visualize(const std::string &file_name)
{
	//pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	// Fill in the cloud data
	cloud->width    = m*m*m;
	cloud->height   = 1;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	grid_index gi;
	
	for (int i = 0; i < cloud->points.size (); i++)
	{
	  this->get_grid_index(i,gi);
	  cloud->points[i].x = gi.i;
	  cloud->points[i].y = gi.j;
	  cloud->points[i].z = gi.k;
	  //std::cout << get_array_index(gi) << ": "<< D[get_array_index(gi)] << std::endl;
	  cloud->points[i].intensity = D[get_array_index(gi)];
	}
	pcl::PolygonMesh output;
	pcl::MarchingCubes<pcl::PointXYZI> *mc;
	mc = new pcl::MarchingCubesSDF<pcl::PointXYZI>;
	mc->setIsoLevel (0.0f);
	mc->setGridResolution (this->m, this->m, this->m);
	mc->setPercentageExtendGrid (0.0f);
	mc->setInputCloud (cloud);
	mc->reconstruct (output);
	
	pcl::io::saveVTKFile(file_name, output);
	//pcl::io::savePCDFileASCII (file_name, cloud);
}
