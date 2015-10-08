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

typedef struct {
  double xrange[2];
  double yrange[2];
  double zrange[2];
} RANGE;

template <typename T>
inline T Sq(const T &x)
{
  return x * x;
}

typedef boost::numeric::ublas::matrix<double> TMatrix;
typedef boost::numeric::ublas::vector<double> TVector;

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
    int num = 40;
    std::vector<TVector> Obstacles;

    double SizeX = 6, SizeY = 5, SizeZ = 5;
    double ObstacleRadius = 0.5;
    double RobotX = 0.5, RobotY = 0.4, RobotZ = 0.3;
    double RobotRadius = sqrt(0.25 * Sq(RobotX) + 0.25 * Sq(RobotY) +0.25 * Sq(RobotZ));

    int selector;

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
#endif