#ifndef PCL_SURFACE_MARCHING_CUBES_SDF_H_
#define PCL_SURFACE_MARCHING_CUBES_SDF_H_

#include <pcl/surface/boost.h>
#include <pcl/surface/marching_cubes.h>

namespace pcl
{
   
  template <typename PointNT>
  class MarchingCubesSDF : public MarchingCubes<PointNT>
  {
    public:
      typedef boost::shared_ptr<MarchingCubesSDF<PointNT> > Ptr;
      typedef boost::shared_ptr<const MarchingCubesSDF<PointNT> > ConstPtr;

      using SurfaceReconstruction<PointNT>::input_;
      using SurfaceReconstruction<PointNT>::tree_;
      using MarchingCubes<PointNT>::grid_;
      using MarchingCubes<PointNT>::res_x_;
      using MarchingCubes<PointNT>::res_y_;
      using MarchingCubes<PointNT>::res_z_;
      using MarchingCubes<PointNT>::min_p_;
      using MarchingCubes<PointNT>::max_p_;

      typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

      typedef typename pcl::KdTree<PointNT> KdTree;
      typedef typename pcl::KdTree<PointNT>::Ptr KdTreePtr;


      MarchingCubesSDF ();

      ~MarchingCubesSDF ();

      void
      voxelizeData ();


    //public:
    //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
//#include <pcl/surface/impl/marching_cubes_hoppe.hpp>
#endif

#endif // PCL_SURFACE_MARCHING_CUBES_SDF_H_

