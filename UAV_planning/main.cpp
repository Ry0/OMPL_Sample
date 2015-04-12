#include "MotionPlanner.h"

int main(){
  Planning planning("../plot/testcase4.dat");
  planning.planWithSimpleSetup();
  // planning.PrintSolution("../plot/UAV.dat", const og::PathGeometric &path);
  planning.OpenGnuplot();
  return 0;
}