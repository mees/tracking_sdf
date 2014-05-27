#include "MarchingCubesSDF.h"
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>
#include <pcl/kdtree/kdtree_flann.h>


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesSDF<PointNT>::MarchingCubesSDF ()
  : MarchingCubes<PointNT> ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubesSDF<PointNT>::~MarchingCubesSDF ()
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubesSDF<PointNT>::voxelizeData ()
{
  //https://github.com/NikolasE/rgbd_utils/blob/master/src/meshing.cpp
  std::cout <<  res_x_<<", " << res_y_ << ", "<< res_z_ << std::endl;
  for (int x = 0; x < res_x_; x++){
    for (int y = 0; y < res_y_; y++){
      for (int z = 0; z < res_z_; z++){
        grid_[x * res_y_*res_z_ + y * res_z_ + z] = input_->points[x * res_y_*res_z_ + y * res_z_ + z].intensity;
      }
    }
  }
}
#define PCL_INSTANTIATE_MarchingCubesSDF(T) template class PCL_EXPORTS pcl::MarchingCubesSDF<T>;