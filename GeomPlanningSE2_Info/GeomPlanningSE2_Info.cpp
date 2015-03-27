/*
Compile: g++ -g -Wall -O2 -march=i686 GeomPlanningSE2_Info.cpp -I ~/prg/ompl/app/ompl/src -I ~/prg/ompl/app/src -L ~/prg/ompl/app/build/lib -lompl -Wl,-rpath ~/prg/ompl/app/build/lib -lboost_thread
Execution: ./a.out
Visualize: plot the following files:
  "path.dat" (sequence of (x,y,yaw)),
  "path0.dat" (sequence of (x,y,yaw); path before simplification),
  "vertices.dat" (set of (x,y,yaw)),
  "edges.dat" (sequence of (x,y,yaw)-(x,y,yaw))
*/

#include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/planners/prm/PRM.h> //PRMプランナを使う場合こっちをインクルード
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

double* xMin;
double* xMax;
double* yMin;
double* yMax;
/// Number of obstacles in space.
int numObstacles;
/// Start position in space
double xStart;
double yStart;
/// Goal position in space
double xGoal;
double yGoal;
/// Max. distance toward each sampled position we
/// should grow our tree
double stepSize;
/// Boundaries of the space
double xLeft;
double xRight;
double yTop;
double yBottom;


void initFromFile(std::string fileName)
{
  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i];
  }

  input >> xStart >> yStart >> xGoal >> yGoal >> stepSize;

  input.close();

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop);

  std::cout << "障害物リスト" << std::endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf]\n", xStart, yStart);
  printf("                        End[%5.2lf, %5.2lf]\n\n", xGoal, yGoal);
}


// Return true if the state is valid, false if the state is invalid
bool isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  for (int i = 0; i < numObstacles; ++i){
    if (x >= xMin[i] && x <= xMax[i] && y >= yMin[i] && y <= yMax[i]){
      return false;
    }
  }
  // Otherwise, the state is valid:
  return true;
}


// Print a vertex to file
void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex)
{
  std::vector<double> reals;
  if(vertex!=ob::PlannerData::NO_VERTEX)// 頂点が存在しない状態じゃなかったら
  {
    space->copyToReals(reals, vertex.getState());// Copy all the real values from a state source to the array reals using getValueAddressAtLocation()
    for(size_t j = 0; j < reals.size(); ++j){
      os << " " << reals[j];
    }
  }
}


void planWithSimpleSetup(void)
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0,xLeft);
  bounds.setHigh(0,xRight);
  bounds.setLow(1,yBottom);
  bounds.setHigh(1,yTop);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(xStart,yStart);
  std::cout << "start: "; start.print(std::cout);

  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(xGoal,yGoal);
  std::cout << "goal: "; goal.print(std::cout);

  ss.setStartAndGoalStates(start, goal);

  ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  std::cout << "----------------" << std::endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(1.0);

  // If we have a solution,
  if (solved)
  {
    // Print the solution path (that is not simplified yet) to a file
    std::ofstream ofs0("../plot/path0.dat");
    ss.getSolutionPath().printAsMatrix(ofs0);

    // Simplify the solution
    ss.simplifySolution();
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(std::cout);

    // Print the solution path to a file
    std::ofstream ofs("../plot/path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss.getSpaceInformation());
    ss.getPlannerData(pdat);

    // Print the vertices to file
    std::ofstream ofs_v("../plot/vertices.dat");
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
      printEdge(ofs_v, ss.getStateSpace(), pdat.getVertex(i));
      ofs_v<<std::endl;
    }

    // Print the edges to file
    std::ofstream ofs_e("../plot/edges.dat");
    std::vector<unsigned int> edge_list;
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
      unsigned int n_edge= pdat.getEdges(i,edge_list);
      for(unsigned int i2(0); i2<n_edge; ++i2)
      {
        printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(i));
        ofs_e<<std::endl;
        printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(edge_list[i2]));
        ofs_e<<std::endl;
        ofs_e<<std::endl<<std::endl;
      }
    }
  }
  else
    std::cout << "No solution found" << std::endl;
}


int main()
{
  initFromFile("../plot/testcase1.dat");
  planWithSimpleSetup();
  return 0;
}
