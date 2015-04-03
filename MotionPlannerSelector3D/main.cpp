#include "MotionPlanner.h"

int main()
{
  Planning planning("../plot/test_arm.dat");
  planning.planWithSimpleSetup();
  planning.OpenGnuplot();
  return 0;
}