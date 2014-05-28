#include "sdf_3d_reconstruction/sdf_mapping.h"

using namespace std;
using namespace Eigen;


//Vector2i project3DPointToImagePlane(Vector3i XYZPoint, cameraMatrix &camera_matrix){
//	//Eq. 2
//	Vector2i ij;
//	float fx = camera_matrix.K(0,0);
//	float fy = camera_matrix.K(1,1);
//	float cx = camera_matrix.K(0,2);
//	float cy = camera_matrix.K(1,2);
//	ij(0) = fx*(XYZPoint(0)/XYZPoint(2)) + cx;
//	ij(1) = fy*(XYZPoint(1)/XYZPoint(2)) + cy;
//	return ij;
//}
//
////incomplete!
//float projectivePointToPointDistance(Matrix<float, 3, 3> &CamRot,
//		Vector3f &CamTrans, grid_index &gi, cameraMatrix &camera_matrix, SDF &old_sdf){
//	/*transfer vertex global coordinates to the local
//	coordinate frame of the camera, Eq. 24 */
//	Vector3i globalCoord;
//	globalCoord(0)=gi.i;
//	globalCoord(1)=gi.j;
//	globalCoord(2)=gi.k;
//	Vector3i localCoord;
//	//fixme: make this work
//	//Vector3i localCoord(CamRot*(globalCoord-CamTrans));
//	//now project this point to the pixel Eq. 25
//	Vector2i ij = project3DPointToImagePlane(localCoord, camera_matrix);
//
//	//compute difference of the depth of the voxel and the
//	//observed depth at (i, j)
//	//float distance = old_sdf.D[old_sdf.get_array_index(gi)] - ;
//	return 0;
//}
//
//

////todo: forget txt file and take TF ground truth
//void readGroundTruthAndUpdateSDF(cameraMatrix &camera_matrix, SDF &old_sdf, std::string &filename) {
//	std::ifstream infile(filename.c_str());
//	std::string line;
//	while (std::getline(infile, line))
//	{
//	    std::istringstream iss(line);
//
//	    float timestamp, tx, ty, tz, qx, qy, qz, qw;
//	    iss >> timestamp >> tx >> ty >> tz >> qx >> qy >>qz >>qw;
//	    Quaternionf quaternion(qx, qy, qz, qw);
//	    Matrix<float, 3, 3> rotMat = quaternion.toRotationMatrix();
//	    Vector3f trans(tx,ty,tz);
//	    updateSDF(camera_matrix, rotMat, trans, old_sdf);
////	    cout<<"trans: "<<trans<<endl;
////	    cout<<"rot: "<<rotMat<<endl;
//	}
//}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sdf_3d_reconstruction");
  SDF_Mapping *sdf_mapping = new SDF_Mapping();


//  std::string groundTruthFile;
//
//    if (ros::param::has("~groundTruthPath"))
//    {
//      ros::param::get("~groundTruthPath", groundTruthFile);
//      cout<<"read groundTruthPath file: "<<groundTruthFile<<endl;
//    }
//    else
//    {
//      cout << "Couldn't locate  ground Truth file" << endl;
//      exit(0);
//    }

//   //fixme: forget txt file and take TF ground truth
//  readGroundTruthAndUpdateSDF(camera_matrix, sdf, groundTruthFile);

  ros::spin ();
}


