/*
Compile: g++ -g -Wall -O2 -march=i686 GeomPlanningSE2_PRM.cpp -I ~/prg/ompl/app/ompl/src -I ~/prg/ompl/app/src -L ~/prg/ompl/app/build/lib -lompl -Wl,-rpath ~/prg/ompl/app/build/lib -lboost_thread
Execution: ./a.out
Visualize: plot "path.dat" (sequence of x,y,yaw)
*/

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/SE2StateSpace.h>
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
    // Simplify the solution
    ss.simplifySolution();
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(std::cout);

    // Print the solution path to a file
    std::ofstream ofs("../plot/path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  }
  else
    std::cout << "No solution found" << std::endl;
}

int main()
{
  planWithSimpleSetup();
  return 0;
}
