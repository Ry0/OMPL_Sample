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
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Return true if the state is valid, false if the state is invalid
bool isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());
  // State is invalid when it is inside a 1x1 box
  // centered at the origin:
  if(std::fabs(x)<0.5 && std::fabs(y)<0.5)
    return false;
  // Otherwise, the state is valid:
  return true;
}

// Print a vertex to file
void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex)
{
  std::vector<double> reals;
  if(vertex!=ob::PlannerData::NO_VERTEX)
  {
    space->copyToReals(reals, vertex.getState());
    for(size_t j(0); j<reals.size(); ++j)  os<<" "<<reals[j];
  }
}

void planWithSimpleSetup(void)
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(-0.9,-0.9);
  std::cout << "start: "; start.print(std::cout);

  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(0.9,0.9);
  std::cout << "goal: "; goal.print(std::cout);

  ss.setStartAndGoalStates(start, goal);

  ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
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
  planWithSimpleSetup();
  return 0;
}
