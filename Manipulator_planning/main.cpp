#include "MotionPlanner.h"
// #include "p4-arm-helper.h"
using namespace std;

int main(){
  Planning planning("../plot/test_arm1.dat");
  planning.planWithSimpleSetup();
  return 0;
}

