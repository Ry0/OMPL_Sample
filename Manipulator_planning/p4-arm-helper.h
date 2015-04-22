#ifndef _P4_ARM_HELPER_H_
#define _P4_ARM_HELPER_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "MotionPlanner.h"
#include "p3-uav-helper.h"

// Compute a rotation matrix from axis (x,y,z) and angle
// 任意の軸中心で回転する姿勢をクォータニオンで表現したあと
// クォータニオン→回転行列に変換
inline TMatrix RfromAxisAngle(const double &x, const double &y, const double &z, const double &angle)
{
  ob::SO3StateSpace::StateType rot;
  rot.setAxisAngle(x,y,z,angle);
  return QtoR(rot);
}

// Compute a rotation matrix from axis (v) and angle
// 任意の方向をベクトルで与えて回転する角度も
inline TMatrix RfromAxisAngle(const TVector &v, const double &angle)
{
  assert(v.size()==3);
  return RfromAxisAngle(v(0), v(1), v(2), angle);
}

#endif