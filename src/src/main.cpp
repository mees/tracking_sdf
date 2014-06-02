#include "sdf_3d_reconstruction/sdf_mapping.h"

using namespace std;
using namespace Eigen;



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sdf_3d_reconstruction");
  SDF_Reconstruction *sdf_mapping = new SDF_Reconstruction();
  //ros::spin ();
}


