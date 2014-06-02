/*
 * sdf.h
 *
 *  Created on: 19.05.2014
 *      Author: joel
 */ 
#ifndef SDF_H_
#define SDF_H_
#include <string>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include<Eigen/Eigen>

#include "sdf_3d_reconstruction/MarchingCubesSDF.h"

using namespace Eigen;
class SDF {
  
private:
	int m;
	float width, height, depth;
	float *D;
	float *W;
	int numberOfVoxels;

public:
	/**
	 *  standard constructor
	 */
	SDF(int m, float width, float height, float depth);
	virtual ~SDF();
	
	/**
	 * gets world coordinates, and returns the interpolated distance
	 */
	float interpolate_distance(float x, float y, float z);
	/**
	 * helper function for testing issues
	 */
	void create_circle(float radius, float center_x, float center_y, float center_z);
	/**
	 * 
	 **/
	void visualize(const std::string &file_name);
	int getNumberOfVoxels();
	int get_array_index(Vector3i& gi);
	void get_grid_index(int array_idx, Vector3i& gi);
};

#endif /* SDF_H_ */
