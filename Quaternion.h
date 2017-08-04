#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Core>


class Quaternion
{
private:

    // quat = a_i + b_j + c_k + w
    Eigen::Vector4d quat;
    Eigen::Matrix4d R_matrix;
   
public:
	// convert an axis-angle representation into a quat
    Quaternion(Eigen::RowVector3d axis, double angle);  // constructor to create a quat

    // compute the 4x4 rotation matrix for a given quat
    Eigen::Matrix4d Quat_to_Rotmatrix();

};

#endif /* QUATERNION_H */
