//#include "sdf_3d_reconstruction/sdf.h"
#include "sdf.h"
#include <Eigen/Dense>
using namespace Eigen;
SDF::SDF(int m, float width, float height, float depth) {
	this->m = m;
	this->width = width;
	this->height = height;
	this->depth = depth;
	// i = 1, j = 2, k = 1, m = 255
	// => idx = k*255^2 + j*255 + i

	// idx = 167045 =>
	// i = (int) idx /(255*255)
	// j = (idx % 255*255)/255
	// k = idx % 255
	this->D = new float[m * m * m];
	this->W = new float[m * m * m];
}

void SDF::create_circle(float radius, float center_x, float center_y,
		float center_z) {
// D ist das einzig wichtige was gefüllt werden muss.
// durch den center point und den Radius wird eine Surface der Kugel impliziert.
// für jeden Punkt im Grid wird ein Wert berechnet, der sich aus
// <euklidischen Distanz zum Center> - <radius> berechnet

// center_x, center_y, center_z sind in Weltkoordinaten
// iteriere über D 3 Schleifen, jeweils über i, j, k
// i, j, k -> x, y, z
// bspw.: z=(depth/m)*k

}
