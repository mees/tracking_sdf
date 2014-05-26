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
  for (int x = 0; x < res_x_; ++x){
    for (int y = 0; y < res_y_; ++y){
      for (int z = 0; z < res_z_; ++z){
        /*std::vector<int> nn_indices;
        std::vector<float> nn_sqr_dists;

        Eigen::Vector3f point;
        point[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * float (x) / float (res_x_);
        point[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * float (y) / float (res_y_);
        point[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * float (z) / float (res_z_);

        PointNT p;
        p.getVector3fMap () = point;

        tree_->nearestKSearch (p, 1, nn_indices, nn_sqr_dists);*/

        grid_[x * res_y_*res_z_ + y * res_z_ + z] = input_->points[x * res_y_*res_z_ + y * res_z_ + z].intensity;
        //std::cout << input_->points[nn_indices[0]].getNormalVector3fMap ().dot (
        //    point - input_->points[nn_indices[0]].getVector3fMap ());
      }
    }
  }
}
#define PCL_INSTANTIATE_MarchingCubesSDF(T) template class PCL_EXPORTS pcl::MarchingCubesSDF<T>;