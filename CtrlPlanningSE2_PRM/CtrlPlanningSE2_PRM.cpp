/*
Compile: g++ -g -Wall -O2 -march=i686 CtrlPlanningSE2.cpp -I ~/prg/ompl/app/ompl/src -I ~/prg/ompl/app/src -L ~/prg/ompl/app/build/lib -lompl -Wl,-rpath ~/prg/ompl/app/build/lib -lboost_thread
Execution: ./a.out
Visualize: plot "path.dat" (sequence of x,y,yaw)
*/

///>>>CHANGE
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
///<<<CHANGE
#include <ompl/base/spaces/SE2StateSpace.h>
///>>>+++
#include <ompl/control/spaces/RealVectorControlSpace.h>
///<<<+++
#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
///>>>CHANGE
namespace oc = ompl::control;
///<<<CHANGE

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

///>>>+++
template<typename T>
inline T FMod(const T &x, const T &y)
{
  if(y==0)  return x;
  return x-y*std::floor(x/y);
}
// Convert radian to [-pi,pi)
double RadToNPiPPi(const double &x)
{
  return FMod(x+M_PI,M_PI*2.0)-M_PI;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
  const ob::SE2StateSpace::StateType *start_2d= start->as<ob::SE2StateSpace::StateType>();
  const double &x(start_2d->getX()), &y(start_2d->getY()), &rot(start_2d->getYaw());
  const oc::RealVectorControlSpace::ControlType *control_2= control->as<oc::RealVectorControlSpace::ControlType>();
  const double &c0((*control_2)[0]), &c1((*control_2)[1]);
  ob::SE2StateSpace::StateType *result_2d= result->as<ob::SE2StateSpace::StateType>();

  double nx= x+c0*duration*std::cos(rot);
  double ny= y+c0*duration*std::sin(rot);
  if(nx<-1.0) nx= -1.0;  else if(nx>1.0) nx= 1.0;
  if(ny<-1.0) ny= -1.0;  else if(ny>1.0) ny= 1.0;
  result_2d->setXY(nx, ny);
  result_2d->setYaw(RadToNPiPPi(rot+c1*duration));
}
///<<<+++

void planWithSimpleSetup(void)
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

///>>>+++
  // Construct the control space
  oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

  ob::RealVectorBounds cbounds(2);
  cbounds.setLow(0,0.0);
  cbounds.setHigh(0,0.5);
  cbounds.setLow(1,-2.0);
  cbounds.setHigh(1,2.0);
  cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
///<<<+++

///>>>CHANGE
  // Instantiate SimpleSetup
  oc::SimpleSetup ss(cspace);
///<<<CHANGE

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

///>>>+++
  // Setup the StatePropagator
  ss.setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));
///<<<+++

  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(-0.9,-0.9);
  start->setYaw(0.25*M_PI);
  std::cout << "start: "; start.print(std::cout);

  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(0.9,0.9);
  goal->setYaw(0.25*M_PI);
  std::cout << "goal: "; goal.print(std::cout);

///>>>CHANGE
  ss.setStartAndGoalStates(start, goal, 0.1);
///<<<CHANGE

///>>>+++
  ss.getSpaceInformation()->setMinMaxControlDuration(1,50);
  ss.getSpaceInformation()->setPropagationStepSize(0.1);
///<<<+++

///>>>CHANGE
  ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));
  ss.setPlanner(planner);
///<<<CHANGE

  std::cout << "----------------" << std::endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(10.0);

  // If we have a solution,
  if (solved)
  {
///>>>---
    // Simplify the solution
    // ss.simplifySolution();
///<<<---
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(std::cout);

    // Print the solution path to a file
    std::ofstream ofs("path.dat");
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
