#include "MotionPlanner.h"
// #include "p4-arm-helper.h"
using namespace std;

int main(){
  MPlanning planning("../plot/testcase4.dat");
  planning.printDistance();

  return 0;
}

