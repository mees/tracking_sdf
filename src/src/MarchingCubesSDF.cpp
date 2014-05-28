
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include "sdf_3d_reconstruction/MarchingCubesSDF.h"
#include "sdf_3d_reconstruction/MarchingCubesSDF.hpp"
#include <pcl/surface/impl/marching_cubes.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(MarchingCubesSDF, (pcl::PointXYZI))
