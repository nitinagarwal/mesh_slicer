#include "Quaternion.h"

Quaternion::Quaternion(Eigen::RowVector3d axis, double angle){

    // angle is in radians
    quat(0) = axis(0) * sin(angle/2);
    quat(1) = axis(1) * sin(angle/2);
    quat(2) = axis(2) * sin(angle/2);
    quat(3) = cos(angle/2);

    quat.normalize();
}

Eigen::Matrix4d Quaternion::Quat_to_Rotmatrix(){

  double xx2 = 2.0f * quat[0] * quat[0];
  double yy2 = 2.0f * quat[1] * quat[1];
  double zz2 = 2.0f * quat[2] * quat[2];
  double xy2 = 2.0f * quat[0] * quat[1];
  double xz2 = 2.0f * quat[0] * quat[2];
  double yz2 = 2.0f * quat[1] * quat[2];
  double wx2 = 2.0f * quat[3] * quat[0];
  double wy2 = 2.0f * quat[3] * quat[1];
  double wz2 = 2.0f * quat[3] * quat[2];

  R_matrix(0,0) = 1.0 - yy2 - zz2;
  R_matrix(0,1) = xy2 - wz2;
  R_matrix(0,2) = xz2 + wy2;
  R_matrix(0,3) = 0;
  R_matrix(1,0) = xy2 + wz2;
  R_matrix(1,1) = 1.0 - xx2 - zz2;
  R_matrix(1,2) = yz2 - wx2;
  R_matrix(1,3) = 0;
  R_matrix(2,0) = xz2 -wy2;
  R_matrix(2,1) = yz2 + wx2;
  R_matrix(2,2) = 1.0 - xx2 - yy2;
  R_matrix(2,3) = 0;
  R_matrix.row(3) << 0, 0, 0, 1;

  /* mat[0*4+0] = - yy2 - zz2 + 1.0f; */
  /* mat[0*4+1] = xy2 + wz2; */
  /* mat[0*4+2] = xz2 - wy2; */
  /* mat[0*4+3] = 0; */
  /* mat[1*4+0] = xy2 - wz2; */
  /* mat[1*4+1] = - xx2 - zz2 + 1.0f; */
  /* mat[1*4+2] = yz2 + wx2; */
  /* mat[1*4+3] = 0; */
  /* mat[2*4+0] = xz2 + wy2; */
  /* mat[2*4+1] = yz2 - wx2; */
  /* mat[2*4+2] = - xx2 - yy2 + 1.0f; */
  /* mat[2*4+3] = 0; */
  /* mat[3*4+0] = mat[3*4+1] = mat[3*4+2] = 0; */
  /* mat[3*4+3] = 1; */

  return R_matrix;
}
