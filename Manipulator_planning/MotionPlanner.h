#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <ostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

template <typename T>
inline T Sq(const T &x)
{
  return x * x;
}

typedef struct {
  double xrange[2];
  double yrange[2];
  double zrange[2];
} RANGE;

typedef boost::numeric::ublas::matrix<double> TMatrix;
typedef boost::numeric::ublas::vector<double> TVector;

struct TLink
{
  TVector Axis;  // Joint axis
  TVector End;   // End-point position of the link defined on the local frame
  TLink(const TVector &a, const TVector &e) : Axis(a), End(e) {}
};

class Planning{
  public:
    Planning(std::string fileName);
    void PlannerSelector();
    void CreateMap();
    void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
    bool isStateValid(const ob::State *state);
    void planWithSimpleSetup();
    void PrintMap();
    void PrintBoxSequence(const char *filename, const og::PathGeometric &path, int skip = 1);
    void PrintSolution(const char *filename, const og::PathGeometric &path, int skip = 1);
    int OpenGnuplot();

  private:
    double RobotX = 0.5, RobotY = 0.4, RobotZ = 0.3;
    double RobotRadius = sqrt(0.25 * Sq(RobotX) + 0.25 * Sq(RobotY) +0.25 * Sq(RobotZ));

  protected:
    double SizeX = 6, SizeY = 5, SizeZ = 5;
    int selector;
    int num = 5;
    std::vector<TVector> Obstacles;
    double ObstacleRadius = 0.5;

    double xStart,yStart,zStart;
    double xGoal,yGoal,zGoal;

    /// Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;
    double zTop;
    double zBottom;
};

class MPlanning:
  public Planning{
    public:
      MPlanning(std::string fileName);
      void planWithSimpleSetup();
      /* Compute the forward kinematics of a manipulator ``linkes''
      whose joint angles are specified by ``angles'',
      and the base position is ``base''.
      The result is stored into ``result'' that contains
      the base position and every position of the end-points.
      i.e. result.size()==linkes.size()+1 */
      void ForwardKinematics(const std::vector<TLink> &linkes, const std::vector<double> &angles, const TVector &base, std::vector<TVector> &result);

      /* Save a sequence of the arm on ``path'' into file that is gnuplot-compatible.
      The path should be a sequence of joint-angles.
      The parameter ``skip'' is an interval to sample from ``path'' (1 for every sample). */
      void PrintArmSequence(const char *filename, const og::PathGeometric &path, int skip=1);
      void CheckArmSequence();
      /* Print the planning result into a file.
      The resulting file is a gnuplot script that plots the path,
      the sequence of the arm on the path, and the obstacles.
      Sequence of the arm on ``path'': stored into "res/frame_all.dat",
      obstacles: stored into the resulting script.
      Usage:  gnuplot -persistent filename */
      void PrintArmSolution(const char *filename, const og::PathGeometric &path, int skip=1);

    private:
      std::vector<TLink> Arm;  // Manipulator
      TVector ArmBase;  // The base position of Manipulator
      double total_len;
  };

#endif