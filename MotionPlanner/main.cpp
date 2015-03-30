#include "MotionPlanner.h"

int main()
{
  MotionPlan::Planning planning("../plot/testcase1.dat");
  planning.planWithSimpleSetup();
  planning.OpenGnuplot();
  return 0;
}