/*
 * sdf.h
 *
 *  Created on: 19.05.2014
 *      Author: joel
 */

#ifndef SDF_H_
#define SDF_H_
struct grid_index{
	int i;
	int j;
	int k;
};
class SDF {
private:
	int m;
	float width, height, depth;
	float D[];
	float W[];
	/**return array access index. Can be used in D or W.
	 * - i: grid x index
	 * - j: grid y index
	 * - k: grid z index
	 */
	int get_array_index(grid_index& gi);
	void get_grid_index(int array_idx, grid_index& gi);
public:
	/**
	 *  standard constructor
	 */
	SDF(int m, float width, float height, float depth);
	/**
	 * gets world coordinates, and returns the interpolated distance
	 */
	float interpolate_distance(float x, float y, float z);
	/**
	 * helper function for testing issues
	 */
	void create_circle(float radius, float center_x, float center_y, float center_z,);
};

#endif /* SDF_H_ */
