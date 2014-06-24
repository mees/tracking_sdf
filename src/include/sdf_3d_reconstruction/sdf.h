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
#include <math.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include<Eigen/Eigen>


#include "sdf_3d_reconstruction/marching_cubes_sdf.h"
#include "sdf_3d_reconstruction/camera_tracking.h"

using namespace Eigen;
class SDF {
  
private:
	int m;
	//width
	float width, height, depth;
	//Distance array
	float *D;
	//Weight array
	float *W;
	//Red Array
	float *R;
	//Green Array
	float *G;
	//Blue Array
	float *B;
	int number_of_voxels;
	ros::Publisher marker_publisher;
	uint32_t shape_type;
	ros::Rate* r;
	void register_visualization();


public:
	/**
	 *  standard constructor
	 */
	SDF(int m, float width, float height, float depth);
	virtual ~SDF();
	
	/**
	 * gets interpolated distance with world coordinates.
	 */
	float interpolate_distance(Vector3d& world_coordinates);
	
	/**
	 * gets interpolated color with world coordinates.
	 */
	void interpolate_color(pcl::PointXYZ& global_coords, std_msgs::ColorRGBA& color);
	/**
	 * helper function for testing issues
	 */
	void create_circle(float radius, float center_x, float center_y, float center_z);
	/**
	 *  
	 **/
	void visualize(const std::string &file_name);
	int get_number_of_voxels();
	/*
	 *  input: i,j,k -> idx. Idx can be used for W and for D
	 */
	int get_array_index(Vector3i& voxel_coordinates);
	/*
	 *  input: array_idx
	 *  output: voxel_coordinates -> i, j, k voxel grid coordinates. i represents x axes, j represents y axes, k represents z axes
	 */
	void get_voxel_coordinates(int array_idx, Vector3i& voxel_coordinates);
	/*
	 *  input: global_coordinates
	 *  output: voxel_coordinates
	 */
	void get_voxel_coordinates(Vector3d& global_coordinates, Vector3i& voxel_coordinates);
	/*
	 *  input: globale coordinate x,y,z
	 *  output: voxel_coordinates i,j,k
	 */
	void get_global_coordinates(Vector3i& voxel_coordinates, Vector3d& global_coordinates);
	/*
	 * 
	 */
	void update(CameraTracking* camera_tracking, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr normals);
};

#endif /* SDF_H_ */
