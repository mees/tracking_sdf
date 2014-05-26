#include "sdf.h"
//#include <Eigen/Dense>
//using namespace Eigen;


SDF::SDF(int m, float width, float height, float depth) {
	this->m = m;
	this->width = width;
	this->height = height;
	this->depth = depth;
	this->D = new float[this->m * this->m * this->m];
	this->W = new float[this->m * this->m * this->m];
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
	for (int array_idx = 0; array_idx < this->m*this->m*this->m; array_idx++){
		this -> get_grid_index(array_idx, gi);
		int x = (this->width/m) * gi.i;
		int y = (this->height/m) * gi.j;
		int z = (this->depth/m) * gi.k;
		float d = (x - center_x)*(x - center_x) + (y - center_y)*(y - center_y)+(z-center_z)*(z-center_z);
		D[array_idx] = radius - d;
	}
}
