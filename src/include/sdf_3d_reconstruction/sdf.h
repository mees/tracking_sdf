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

#include<Eigen/Eigen>
#include <omp.h>

#include "sdf_3d_reconstruction/marching_cubes_sdf.h"
#include "sdf_3d_reconstruction/camera_tracking.h"
#include <condition_variable>
#include <thread>
#include <atomic>

using namespace Eigen;
class SDF {

private:
	//width
	float width, height, depth, distance_delta, distance_epsilon;
	//Distance array
	float *D;
	//Weight array
	float *W;
	//global coordinates of the voxels
	Vector3d *global_coords;

	Vector3i *voxel_coords;

	float *Color_W;
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
	Vector3d sdf_origin;
	pcl::MarchingCubesSDF *mc;
	bool initial_update_done;
	std::condition_variable cv;
	std::mutex cv_m;
	int m_squared;

public:
	int m;
	float m_div_height;
	float m_div_width;
	float m_div_depth;
	std::atomic_bool finish_visualization_thread;

	/**
	 *  standard constructor
	 */
	SDF(int m, float width, float height, float depth, Vector3d& sdf_origin,
			float distance_delta, float distance_epsilon);
	virtual ~SDF();

	/**
	 * gets interpolated distance with world coordinates.
	 */

	float interpolate_distance(const Vector3d& world_coordinates, bool& is_interpolated) const;

	/**
	 * gets interpolated color with world coordinates.
	 */
	void interpolate_color(geometry_msgs::Point& global_coords,
			std_msgs::ColorRGBA& color) const;
	/**
	 * helper function for testing issues
	 */
	void create_circle(float radius, float center_x, float center_y,
			float center_z);
	/**
	 * helper function for testing issues
	 */
	void create_cuboid(float min_x, float max_x, float min_y, float max_y,
			float min_z, float max_z);
	/**
	 *  
	 **/
	void visualize(double frequency);
	inline int get_number_of_voxels() const {
		return number_of_voxels;
	}
	/*
	 *  input: i,j,k -> idx. Idx can be used for W and for D
	 */
	inline int get_array_index(Vector3i& voxel_coordinates) const{
	        if (voxel_coordinates(0) < 0 || voxel_coordinates(1) < 0 || voxel_coordinates(2) < 0){
		  return -1;
	        }
	        if (voxel_coordinates(0) >= m || voxel_coordinates(1) >= m || voxel_coordinates(2) >= m){
		  return -1;
	        }
		int _idx = m_squared*voxel_coordinates(0)+this->m*voxel_coordinates(1)+voxel_coordinates(2);
		if (_idx < 0 || _idx >= number_of_voxels){
		  std::cout << "Error in get_array_index \n"<< voxel_coordinates << std::endl;
		  _idx = -1;
		}

		return _idx;
	}
	/*
	 *  input: array_idx
	 *  output: voxel_coordinates -> i, j, k voxel grid coordinates. i represents x axes, j represents y axes, k represents z axes
	 */
	inline void get_voxel_coordinates(int array_idx, Vector3i& voxel_coordinates) const {
		voxel_coordinates(1) = (int) (array_idx % m_squared)/this->m;
		voxel_coordinates(0) = (int) (array_idx/m_squared);
		voxel_coordinates(2) = (int) array_idx%this->m;
	}


	/*
	 *  input: global_coordinates
	 *  output: voxel_coordinates
	 */
	inline void get_voxel_coordinates(Vector3d& global_coordinates, Vector3d& voxel_coordinates)  const{
		voxel_coordinates(0) = ((global_coordinates(0)-this->sdf_origin(0))*m_div_width -0.5);
		voxel_coordinates(1) = ((global_coordinates(1)-this->sdf_origin(1))*m_div_height -0.5);
		voxel_coordinates(2) = ((global_coordinates(2)-this->sdf_origin(2))*m_div_depth -0.5);
	}
	/*
	 *  input: globale coordinate x,y,z
	 *  output: voxel_coordinates i,j,k
	 */
	// auf dem Blatt verifiziert durch Oier und Joel
	inline void get_global_coordinates(Vector3i& voxel_coordinates, Vector3d& global_coordinates) const{
		global_coordinates(0) = (this->width/((float)m)) * (voxel_coordinates(0)+0.5)+this->sdf_origin(0);
		global_coordinates(1) = (this->height/((float)m)) * (voxel_coordinates(1)+0.5)+this->sdf_origin(1);
		global_coordinates(2) = (this->depth/((float)m)) * (voxel_coordinates(2)+0.5)+this->sdf_origin(2);
	}
	/*
	 * 
	 */
	void update(CameraTracking* camera_tracking,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered,
			pcl::PointCloud<pcl::Normal>::Ptr normals);

	/*
	 * point-to-point distance computes the difference of the depth of the voxel
	 *  and the observed depth of the projected voxel pixel in the depth image, in camera frame
	 */
	inline void projectivePointToPointDistance(const double &voxelDepthInCameraFrame, const double &observedDepthOfProjectedVoxelInDepthImage, double &pointToPointDistance) const {

		pointToPointDistance = voxelDepthInCameraFrame - observedDepthOfProjectedVoxelInDepthImage;
	}
	/*
	 * point-to-point distance computes the difference of the depth of the voxel
	 *  and the observed depth of the projected voxel pixel in the depth image, in camera frame
	 */
	inline void projectivePointToPlaneDistance(const Vector3d &camera_point, const Vector3d &camera_point_img, const Vector3d &normal, double &pointToPlaneDistance) const{
		Vector3d diff_vec = camera_point_img-camera_point;
		pointToPlaneDistance = diff_vec.dot(normal);

	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif /* SDF_H_ */
