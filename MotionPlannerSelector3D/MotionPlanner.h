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

class Planning{
  public:
    Planning(std::string fileName);
    void initFromFile(std::string fileName);
    void CreateCube();
    void PlannerSelector();
    void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
    bool isStateValid(const ob::State *state);
    void planWithSimpleSetup();
    void output_plt(std::string plt_output);
    int OpenGnuplot();

  private:
    double* xMin;
    double* xMax;
    double* yMin;
    double* yMax;
    double* zMin;
    double* zMax;
    // Number of obstacles in space.
    int numObstacles;
    // Start position in space
    double xStart;
    double yStart;
    double zStart;
    // Goal position in space
    double xGoal;
    double yGoal;
    double zGoal;
    // Max. distance toward each sampled position we
    // should grow our tree
    double stepSize;
    // Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;
    double zTop;
    double zBottom;

    int selector;
};
#endif