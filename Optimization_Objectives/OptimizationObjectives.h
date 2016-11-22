#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/prm/PRM.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/planners/rrt/LBTRRT.h>
// #include <ompl/geometric/planners/rrt/LazyRRT.h>
// #include <ompl/geometric/planners/rrt/RRT.h>
// #include <ompl/geometric/planners/rrt/RRTConnect.h>
// #include <ompl/geometric/planners/rrt/TRRT.h>
// #include <ompl/geometric/planners/rrt/pRRT.h>
// #include <ompl/geometric/planners/est/EST.h>

// #include <ompl/base/spaces/SE2StateSpace.h>
// #include <ompl/base/PlannerData.h>
// #include <cmath>
// #include <iostream>
// #include <fstream>
// #include <ostream>

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
#endif