#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <iostream>
#include <stdio.h>
#include <time.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace base
  {
    class PathLengthOptimizationObjectiveMod : public ob::PathLengthOptimizationObjective
    {
      public:
        PathLengthOptimizationObjectiveMod(const ob::SpaceInformationPtr& si) : ob::PathLengthOptimizationObjective(si){
        }
        ob::Cost stateCost(const ob::State* s) const{
          ob::Cost tmp;
          tmp = identityCost();
          std::cout << "stateCost = " << tmp.value() << std::endl;
          return identityCost();
        }
        ob::Cost motionCost(const State *s1, const State *s2) const
        {
          ob::Cost tmp;
          tmp = Cost(si_->distance(s1, s2));
          std::cout << "motionCost = " << tmp.value() << std::endl;
          return tmp;
        }
    };
  }
}

namespace ompl
{
  namespace base
  {
    class MechanicalWorkOptimizationObjectiveMod : public ob::MechanicalWorkOptimizationObjective
    {
      public:
        MechanicalWorkOptimizationObjectiveMod(const ob::SpaceInformationPtr& si, double pathLengthWeight) : ob::MechanicalWorkOptimizationObjective(si, pathLengthWeight){
          srand((unsigned int)std::time(NULL));
        }
        double getPathLengthWeight() const
        {
          std::cout << "pathLengthWeight_ = " << pathLengthWeight_ << std::endl;
          return pathLengthWeight_;
        }

        ob::Cost stateCost(const State *s) const
        {
          double rnd;
          rnd = (rand()%1000+1)/1000.0;
          ob::Cost tmp = Cost(rnd);
          std::cout << "stateCost = " << tmp.value() << std::endl;
          return Cost(1.0);
        }

        ob::Cost motionCost(const State *s1, const State *s2) const
        {
          // Only accrue positive changes in cost
          double positiveCostAccrued = std::max(stateCost(s2).value() - stateCost(s1).value(), 0.0);
          ob::Cost tmp;
          tmp = Cost(positiveCostAccrued + pathLengthWeight_ * si_->distance(s1, s2));
          std::cout << "motionCost = " << tmp.value() << std::endl;
          return tmp;
        }
    };
  }
}
#endif