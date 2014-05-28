/*
 * sdf.h
 *
 *  Created on: 19.05.2014
 *      Author: joel
 */ 
#ifndef SDF_H_
#define SDF_H_
#include <string>
struct grid_index{
	int i;
	int j;
	int k;
};
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
	int get_array_index(grid_index& gi);
	void get_grid_index(int array_idx, grid_index& gi);
};

#endif /* SDF_H_ */
