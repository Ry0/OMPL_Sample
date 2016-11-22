#include "MotionPlanner.h"


int main()
{
  Planning planning("../plot/testcase1.dat");
  planning.planWithSimpleSetup();
  planning.OpenGnuplot();
  return 0;
}