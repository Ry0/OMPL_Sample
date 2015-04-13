#include "MotionPlanner.h"

int main(){
  Planning planning("../plot/testcase4.dat");
  planning.planWithSimpleSetup();
  planning.OpenGnuplot();
  return 0;
}