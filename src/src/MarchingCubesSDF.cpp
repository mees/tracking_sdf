
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include "MarchingCubesSDF.h"
#include "MarchingCubesSDF.hpp"
#include <pcl/surface/impl/marching_cubes.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE(MarchingCubesSDF, (pcl::PointXYZI))
