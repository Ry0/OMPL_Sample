#include "MotionPlanner.h"
using namespace std;

int main(){
  MPlanning planning("../plot/testcase4.dat");
  planning.CheckArmSequence();
  return 0;
}

