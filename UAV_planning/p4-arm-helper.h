// Utility for the manipulator (arm) planning problem

#include "p3-uav-helper.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

struct TLink
{
  TVector Axis;  // Joint axis
  TVector End;   // End-point position of the link defined on the local frame
  TLink(const TVector &a, const TVector &e) : Axis(a), End(e) {}
};
std::vector<TLink> Arm;  // Manipulator
TVector ArmBase;  // The base position of Manipulator

// Compute a rotation matrix from axis (x,y,z) and angle
inline TMatrix RfromAxisAngle(const double &x, const double &y, const double &z, const double &angle)
{
  ob::SO3StateSpace::StateType rot;
  rot.setAxisAngle(x,y,z, angle);
  return QtoR(rot);
}
// Compute a rotation matrix from axis (v) and angle
inline TMatrix RfromAxisAngle(const TVector &v, const double &angle)
{
  assert(v.size()==3);
  return RfromAxisAngle(v(0), v(1), v(2), angle);
}

/* Compute the forward kinematics of a manipulator ``linkes''
    whose joint angles are specified by ``angles'',
    and the base position is ``base''.
    The result is stored into ``result'' that contains
    the base position and every position of the end-points.
    i.e. result.size()==linkes.size()+1 */
void ForwardKinematics(const std::vector<TLink> &linkes, const std::vector<double> &angles, const TVector &base, std::vector<TVector> &result)
{
  assert(linkes.size()==angles.size());
  assert(base.size()==3);
  result.resize(linkes.size()+1);
  TMatrix R(3,3),Rtmp(3,3);
  R(0,0)=R(1,1)=R(2,2)= 1.0;
  R(0,1)=R(0,2)=R(1,0)=R(1,2)=R(2,0)=R(2,1)= 0.0;
  result[0]= base;
  for(size_t i(1); i<result.size(); ++i)
  {
    result[i].resize(3);
    Rtmp= prod(R,RfromAxisAngle(linkes[i-1].Axis, angles[i-1]));
    R= Rtmp;
    result[i]= prod(R,linkes[i-1].End) + result[i-1];
  }
}

/* Save a sequence of the arm on ``path'' into file that is gnuplot-compatible.
    The path should be a sequence of joint-angles.
    The parameter ``skip'' is an interval to sample from ``path'' (1 for every sample). */
void PrintArmSequence(const char *filename, const og::PathGeometric &path, int skip=1)
{
  using namespace std;
  using namespace boost::numeric::ublas;
  ofstream ofs(filename);
  std::vector<double> angles(Arm.size());
  std::vector<TVector> jpos;
  for(size_t i(0); i<path.getStateCount(); i+=skip)
  {
    const ob::RealVectorStateSpace::StateType *s= path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    for(size_t i(0); i<Arm.size(); ++i)
      angles[i]= (*s)[i];
    ForwardKinematics(Arm, angles, ArmBase, jpos);
    for(size_t i(0); i<jpos.size(); ++i)
      ofs<<jpos[i]<<endl;
    ofs<<endl<<endl;
  }
}

/* Print the planning result into a file.
    The resulting file is a gnuplot script that plots the path,
    the sequence of the arm on the path, and the obstacles.
    Sequence of the arm on ``path'': stored into "res/frame_all.dat",
    obstacles: stored into the resulting script.
    Usage:  gnuplot -persistent filename */
void PrintArmSolution(const char *filename, const og::PathGeometric &path, int skip=1)
{
  using namespace std;
  ofstream ofs(filename);
  PrintArmSequence("res/frame_all.dat", path, skip);
  ofs<<"\
#set terminal png size 800, 640 transparent                     \n\
#set terminal svg size 1200 780 fname 'Trebuchet MS' fsize 24   \n\
set xlabel 'x'         \n\
set ylabel 'y'         \n\
set zlabel 'z'         \n\
set hidden3d           \n\
set ticslevel 0        \n\
set size 0.7,1         \n\
set parametric         \n\
set urange [0:6.28]    \n\
set vrange [0:6.28]    \n\
set isosample 8,8      \n\
set samples 10         \n\
r= "<<ObstacleRadius<<endl;
  ofs<<"splot \\"<<endl;
  for(vector<TVector>::const_iterator itr(Obstacles.begin()),last(Obstacles.end()); itr!=last; ++itr)
  {
    const double &ox((*itr)(0)), &oy((*itr)(1)), &oz((*itr)(2));
    ofs<<"  "<<"r*cos(u)*cos(v)+"<<ox
        <<",r*sin(u)*cos(v)+"<<oy
        <<",r*sin(v)+"<<oz<<" w l lt 1 lw 0.2 t '',"<<"\\"<<endl;
  }
  ofs<<"'res/frame_all.dat' w lp lt 3 pt 6 lw 1.5"<<endl;
}
