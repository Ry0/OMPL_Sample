#ifndef _P3_UAV_HELPER_H_
#define _P3_UAV_HELPER_H_

#include "MotionPlanner.h"

// Generate a vector with 3-dim
inline TVector V3(const double &x, const double &y, const double &z)
{
  TVector v(3);
  v(0) = x;
  v(1) = y;
  v(2) = z;
  return v;
}
// Convert a quaternion to a rotation matrix
// http://simlon.co.jp/quaternion.html
inline TMatrix QtoR(const double &qx, const double &qy, const double &qz, const double &qw)
{
  TMatrix M(3, 3);
  M(0, 0) = qw * qw + qx * qx - qy * qy - qz * qz;
  M(0, 1) = 2.0 * (qx * qy - qw * qz);
  M(0, 2) = 2.0 * (qx * qz + qw * qy);
  M(1, 0) = 2.0 * (qx * qy + qw * qz);
  M(1, 1) = qw * qw - qx * qx + qy * qy - qz * qz;
  M(1, 2) = 2.0 * (qy * qz - qw * qx);
  M(2, 0) = 2.0 * (qx * qz - qw * qy);
  M(2, 1) = 2.0 * (qy * qz + qw * qx);
  M(2, 2) = qw * qw - qx * qx - qy * qy + qz * qz;
  return M;
}

// Convert a OMPL's quaternion to a rotation matrix
inline TMatrix QtoR(const ob::SO3StateSpace::StateType &rot)
{
  return QtoR(rot.x, rot.y, rot.z, rot.w);
}

// Standard quaternion multiplication: q= q0 * q1
inline void QuaternionProduct(ob::SO3StateSpace::StateType &q,
                              const ob::SO3StateSpace::StateType &q0,
                              const ob::SO3StateSpace::StateType &q1)
{
  q.x = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y;
  q.y = q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z;
  q.z = q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x;
  q.w = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z;
}

std::ostream &operator<<(std::ostream &os, const TVector &x)
{
  for (std::size_t i(0); i < x.size(); ++i) os << " " << x(i);
  return os;
}

#endif