#include "MotionPlanner.h"

#include "p3-uav-helper.h"
#include "p4-arm-helper.h"

using namespace std;

// コンストラクタ
Planning::Planning(std::string fileName)
{
  initFromFile(fileName);
  SetArm();
  PlannerSelector();
  srand((unsigned int)time(NULL));
}



void Planning::SetArm(){
  // ArmBase: マニピュレータのベース位置．以下のように設定する：
  ArmBase = V3(0.0,0.0,0.0);

  // Arm: マニピュレータオブジェクト．実質，ローカルフレームで定義された関節の方向ベクトルと，エンドポイント（端点）のベクトルから構成される構造体のベクトルである．以下のように，多リンク系を作る：
  total_len = 0.99 * (SizeZ - ArmBase(2));
  Arm.push_back(TLink(V3(0,0,1), V3(0,0,0.0)));
  Arm.push_back(TLink(V3(0,1,0), V3(0,0,total_len/(double)3)));
  Arm.push_back(TLink(V3(0,1,0), V3(0,0,total_len/(double)3)));
  Arm.push_back(TLink(V3(0,1,0), V3(0,0,total_len/(double)3)));
}



void Planning::PlannerSelector()
{
  std::string plan[9] = {"PRM",    "RRT",     "RRTConnect", "RRTstar",
                         "LBTRRT", "LazyRRT", "TRRT",       "pRRT",
                         "EST"};
  std::string yn;
  while (1) {
    cout << "プランナーを選択してください" << endl;

    printf("PRM        → 1\n");
    printf("RRT        → 2\n");
    printf("RRTConnect → 3\n");
    printf("RRTstar    → 4\n");
    printf("LBTRRT     → 5\n");
    printf("LazyRRT    → 6\n");
    printf("TRRT       → 7\n");
    printf("pRRT       → 8\n");
    printf("EST        → 9\n");

    while (1) {
      cout << "数字を入力 >>";
      cin >> selector;
      if(1 <= selector && selector <= 9){
        break;
      }
    }

    cout << plan[selector - 1] << "プランナーを使います よろしいですか？(y/n)" << endl;
    cin >> yn;
    if(yn == "y"){
      break;
    }
  }
}



void Planning::initFromFile(std::string fileName)
{
  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> zBottom >> zTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];
  zMin = new double[numObstacles];
  zMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i] >> zMin[i] >> zMax[i];
  }

  input >> xStart >> yStart >> zStart >> xGoal >> yGoal >> zGoal;

  input.close();

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop, zBottom, zTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf] z[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i], zMin[i], zMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf, %5.2lf]\n", xStart, yStart, zStart);
  printf("                        End[%5.2lf, %5.2lf, %5.2lf]\n\n", xGoal, yGoal, zGoal);
}



void Planning::CreateCube(std::ostream &cube)
{
  RANGE obstacle;

  for(int ob = 0; ob < numObstacles; ++ob){
    obstacle.xrange[0] = xMin[ob]; obstacle.yrange[0] = yMin[ob]; obstacle.zrange[0] = zMin[ob];
    obstacle.xrange[1] = xMax[ob]; obstacle.yrange[1] = yMax[ob]; obstacle.zrange[1] = zMax[ob];
    for (int i = 0; i < 2; ++i){
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << "\t" << obstacle.zrange[i] << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << obstacle.zrange[i] << std::endl;
      cube << "\n\n";
    }

    for (int i = 0; i < 2; ++i){
      for (int j = 0; j < 2; ++j){
        for (int k = 0; k < 2; ++k){
          cube << obstacle.xrange[i] << "\t" << obstacle.yrange[j] << "\t" << obstacle.zrange[k] << std::endl;
        }
        cube << "\n\n";
      }
    }
  }
}



// (xTest, yTest)が障害物の中にあるかどうかの判定
bool Planning::clear(const double* xMin, const double* xMax,
                     const double* yMin, const double* yMax,
                     const double* zMin, const double* zMax,
                     int numObstacles,
                     double xTest, double yTest, double zTest)
{
  for (int i = 0; i < numObstacles; ++i) { // 障害物の範囲内ならreturn false
    if (xMin[i] <= xTest && xTest <= xMax[i] &&
        yMin[i] <= yTest && yTest <= yMax[i] &&
        zMin[i] <= zTest && zTest <= zMax[i]) {
      return false;
    }
  }

  return true; // すべての障害物の中に入ってなかったらreturn true
}



void Planning::CreateGridPoint(const double* xMin, const double* xMax,
                               const double* yMin, const double* yMax,
                               const double* zMin, const double* zMax, POINT *P, int i)
{
  P[0].x = xMin[i]; P[0].y = yMin[i]; P[0].z = zMin[i]; // x,y,zの最小値、最大値からそれぞれの座標を割り出し、Pに代入。
  P[1].x = xMax[i]; P[1].y = yMin[i]; P[1].z = zMin[i];
  P[2].x = xMax[i]; P[2].y = yMax[i]; P[2].z = zMin[i];
  P[3].x = xMin[i]; P[3].y = yMax[i]; P[3].z = zMin[i];
  P[4].x = xMin[i]; P[4].y = yMin[i]; P[4].z = zMax[i];
  P[5].x = xMax[i]; P[5].y = yMin[i]; P[5].z = zMax[i];
  P[6].x = xMax[i]; P[6].y = yMax[i]; P[6].z = zMax[i];
  P[7].x = xMin[i]; P[7].y = yMax[i]; P[7].z = zMax[i];
}



void Planning::PlaneEquation(POINT p[], int i0[], int i1[], int i2[], int i, double a[])
{
  // 平面の方程式の係数を導出 (ax+by+cz=d)
  // http://keisan.casio.jp/has10/SpecExec.cgi
  a[0] = (p[i1[i]].y-p[i0[i]].y)*(p[i2[i]].z-p[i0[i]].z)-(p[i2[i]].y-p[i0[i]].y)*(p[i1[i]].z-p[i0[i]].z);
  a[1] = (p[i1[i]].z-p[i0[i]].z)*(p[i2[i]].x-p[i0[i]].x)-(p[i2[i]].z-p[i0[i]].z)*(p[i1[i]].x-p[i0[i]].x);
  a[2] = (p[i1[i]].x-p[i0[i]].x)*(p[i2[i]].y-p[i0[i]].y)-(p[i2[i]].x-p[i0[i]].x)*(p[i1[i]].y-p[i0[i]].y);
  a[3] = a[0]*p[i0[i]].x + a[1]*p[i0[i]].y + a[2]*p[i0[i]].z;

  //cout << a[0] << " x + " << a[1] << " y + " << a[2] <<  " z + " << a[3] << " = 0" << endl;
}



void Planning::Pcompare(POINT A, POINT B, POINT *compare)
{
  // 与えられた2点のどちらが小さいか、配列compareに小さい順に格納
  if ((A.x - B.x) > 0) {
    compare[0].x = B.x;
    compare[1].x = A.x;
  }else{
    compare[0].x = A.x;
    compare[1].x = B.x;
  }

  if((A.y - B.y) > 0) {
    compare[0].y = B.y;
    compare[1].y = A.y;
  }else{
    compare[0].y = A.y;
    compare[1].y = B.y;
  }

  if((A.z - B.z) > 0) {
    compare[0].z = B.z;
    compare[1].z = A.z;
  }else{
    compare[0].z = A.z;
    compare[1].z = B.z;
  }
}



bool Planning::link(const double* xMin, const double* xMax,
                    const double* yMin, const double* yMax,
                    const double* zMin, const double* zMax,
                    int numObstacles,
                    double xStart, double yStart, double zStart,
                    double xDest, double yDest, double zDest)
{
  int i = 0;
  bool flag = true;
  POINT A, B, P, p[8], E, compare[2];
  double a[4];
  double t, DE;
  int i0[6] = {0,0,1,2,3,4}; // 配列の組み合わせはノート見る。
  int i1[6] = {1,1,2,3,4,5}; // 0から5までの6面定義、3点で考える。
  int i2[6] = {2,4,5,6,7,6};

  A.x = xStart; A.y = yStart; A.z = zStart;
  B.x = xDest ; B.y = yDest ; B.z = zDest;

  if (!clear(xMin, xMax, yMin, yMax, zMin, zMax, numObstacles, xStart, yStart, zStart) ||
      !clear(xMin, xMax, yMin, yMax, zMin, zMax, numObstacles, xDest, yDest, zDest)) {
    return false;
  }

  Pcompare(A, B, compare);

  // 平面と直線の交点を導出 (参考サイト：http://www.hiramine.com/programming/graphics/3d_planesegmentintersection.html)
  E.x = A.x - B.x;
  E.y = A.y - B.y;
  E.z = A.z - B.z;

  for (int j = 0; j < numObstacles; ++j) {
    CreateGridPoint(xMin, xMax, yMin, yMax, zMin, zMax, p, j);

    do{
      PlaneEquation(p, i0, i1, i2, i, a);

      t = (a[3] - (a[0] * A.x + a[1] * A.y + a[2] * A.z)) / (a[0] * E.x + a[1] * E.y + a[2] * E.z);
      DE = a[0] * E.x + a[1] * E.y + a[2] * E.z;

      if(DE == 0){
        // std::cout << j+1 << "番目の障害物: " << i << "面と直線との交点なし" << std::endl;
      }else{
        // std::cout << "交点あり" << endl;
        P.x = A.x + t * E.x;
        P.y = A.y + t * E.y;
        P.z = A.z + t * E.z;
        // std::cout << j+1 << "番目の障害物: " << i << "面, (" << P.x << ", " << P.y << ", " << P.z << ")" << std::endl;

        if (xMin[j] <= P.x && P.x <= xMax[j] &&
            yMin[j] <= P.y && P.y <= yMax[j] &&
            zMin[j] <= P.z && P.z <= zMax[j] &&
            compare[0].x <= P.x && P.x <= compare[1].x &&
            compare[0].y <= P.y && P.y <= compare[1].y &&
            compare[0].z <= P.z && P.z <= compare[1].z) {
          // std::cout << j+1 << "番目の障害物: " << i << "面, 定義した4点内に入っている" << std::endl;
          flag = false;
          break;
        }else{
          // std::cout << j+1 << "番目の障害物: " << i << "面, 範囲外" << std::endl;
        }
      }
      i++;
    }while(i<6);
    i = 0;
  }

  return flag;
}



bool Planning::isStateValid(const ob::State *state)
{
  // const ob::RealVectorStateSpace::StateType *state_vec= state->as<ob::RealVectorStateSpace::StateType>();
  // std::vector<double> angles;
  // std::vector<TVector> result;

  // for (int i = 0; i < num; ++i){
  //   angles.push_back(state_vec->values[i]);
  // }

  // ForwardKinematics(Arm, angles, ArmBase, result);
  // for(size_t i = 0; i < result.size(); ++i){
  //   cout << result[i] << endl;
  // }
  // cout << endl << endl;

  // for (size_t i = 0; i < result.size()-1; ++i){
  //   if(link(xMin, xMax, yMin, yMax, zMin, zMax, numObstacles,
  //           result[i](0), result[i](1), result[i](2),
  //           result[i+1](0), result[i+1](1), result[i+1](2))==false){
  //     cout << "衝突してます" << endl;
  //     return false;
  //   }
  // }
  double roulette;
  roulette = (((double)rand())/RAND_MAX)*100;
  if(roulette>80){
    return true;
  }else{
    return false;
  }


}



/* Compute the forward kinematics of a manipulator ``linkes''
whose joint angles are specified by ``angles'',
and the base position is ``base''.
The result is stored into ``result'' that contains
the base position and every position of the end-points.

i.e. result.size()==linkes.size()+1 */
void Planning::ForwardKinematics(const std::vector<TLink> &linkes,
                                  const std::vector<double> &angles, const TVector &base,
                                  std::vector<TVector> &result)
{
  assert(linkes.size() == angles.size());
  assert(base.size() == 3);
  result.resize(linkes.size() + 1);
  TMatrix R(3, 3), Rtmp(3, 3);
  R(0, 0) = R(1, 1) = R(2, 2) = 1.0;
  R(0, 1) = R(0, 2) = R(1, 0) = R(1, 2) = R(2, 0) = R(2, 1) = 0.0;
  result[0] = base;
  for (size_t i(1); i < result.size(); ++i) {
    result[i].resize(3);
    Rtmp = prod(R, RfromAxisAngle(linkes[i - 1].Axis, angles[i - 1]));
    R = Rtmp;
    result[i] = prod(R, linkes[i - 1].End) + result[i - 1];
  }
}



// Print a vertex to file
void Planning::printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex)
{
  std::vector<double> reals;
  if(vertex!=ob::PlannerData::NO_VERTEX)// 頂点が存在しない状態じゃなかったら
  {
    space->copyToReals(reals, vertex.getState());  // Copy all the real values from a state source
                                                   // to the array reals using
                                                   // getValueAddressAtLocation()
    for (size_t j = 0; j < reals.size(); ++j) {
      os << " " << reals[j];
    }
  }
}


void Planning::planWithSimpleSetup()
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::RealVectorStateSpace(4));

  int count = 0;

  ob::RealVectorBounds bounds(4);
  for (int i = 0; i < 4; ++i){
    bounds.setLow(i, -M_PI);
    bounds.setHigh(i, M_PI);
  }
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&Planning::isStateValid, this, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::RealVectorStateSpace> start(space);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = M_PI/10;
  start->as<ob::RealVectorStateSpace::StateType>()->values[2] = M_PI/2;
  start->as<ob::RealVectorStateSpace::StateType>()->values[3] = -M_PI/2;
  // start->rotation().setIdentity();
  // start.random();
  cout << "start: ";
  start.print(cout);

  ob::ScopedState<ob::RealVectorStateSpace> goal(space);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = M_PI/2;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = M_PI/10;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[3] = M_PI/2;
  // goal->setXYZ(xGoal,yGoal,zGoal);
  // goal->rotation().setIdentity();
  // goal.random();
  cout << "goal: ";
  goal.print(cout);

  ss.setStartAndGoalStates(start, goal);

  if (selector == 1) {
    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 2) {
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 3) {
    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 4) {
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 5) {
    ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 6) {
    ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 7) {
    ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 8) {
    ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  } else if (selector == 9) {
    ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }

  cout << "----------------" << endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(20);

  while (1) {
    // If we have a solution,
    if (solved) {
      // Print the solution path (that is not simplified yet) to a file
      std::ofstream ofs0("../plot/path0.dat");
      ss.getSolutionPath().printAsMatrix(ofs0);

      // Simplify the solution
      ss.simplifySolution();
      cout << "----------------" << endl;
      cout << "Found solution:" << endl;
      // Print the solution path to screen
      ss.getSolutionPath().print(cout);

      // Print the solution path to a file
      std::ofstream ofs("../plot/path.dat");
      ss.getSolutionPath().printAsMatrix(ofs);
      OpenGnuplot("../plot/frame_all.dat", ss.getSolutionPath());

      // Get the planner data to visualize the vertices and the edges
      ob::PlannerData pdat(ss.getSpaceInformation());
      ss.getPlannerData(pdat);

      // Print the vertices to file
      std::ofstream ofs_v("../plot/vertices.dat");
      for (unsigned int i(0); i < pdat.numVertices(); ++i) {
        printEdge(ofs_v, ss.getStateSpace(), pdat.getVertex(i));
        ofs_v << endl;
      }

      // Print the edges to file
      std::ofstream ofs_e("../plot/edges.dat");
      std::vector<unsigned int> edge_list;
      for (unsigned int i(0); i < pdat.numVertices(); ++i) {
        unsigned int n_edge = pdat.getEdges(i, edge_list);
        for (unsigned int i2(0); i2 < n_edge; ++i2) {
          printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(i));
          ofs_e << endl;
          printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(edge_list[i2]));
          ofs_e<<endl;
          ofs_e<<endl<<endl;
        }
      }
      break;
    } else {
      cout << "No solution found" << endl;
      count++;
      if(count > 3){
        cout << "全然経路見つからんし！" << endl;
        break;
      }
    }

  }
}




// 参考：http://www-sens.sys.es.osaka-u.ac.jp/wakate/tutorial/group3/gnuplot/
int Planning::OpenGnuplot(const char *filename, const og::PathGeometric &path, int skip)
{
  // output_plt("../plot/plot.plt");
  PrintArmSequence(filename, path, skip);

  FILE *fp = popen("cd ../plot && gnuplot -persist", "w");
  if (fp == NULL) {
    return -1;
  }
  ofstream ofs("../plot/test.dat");
  fputs("set mouse\n", fp);
  CreateCube(ofs);
  // fputs("splot \"test.dat\" using 1:2:3 with lines,\n", fp);
  fputs("splot \"frame_all.dat\" w lp lt 3 pt 6 lw 1.5\n", fp);
  // fputs("load \"ARM.plt\"\n", fp);

  fflush(fp);
  cin.get();
  pclose(fp);
  return 0;
}


/* Save a sequence of the arm on ``path'' into file that is gnuplot-compatible.
   The path should be a sequence of joint-angles.
   The parameter ``skip'' is an interval to sample from ``path'' (1 for every sample). */
void Planning::PrintArmSequence(const char *filename, const og::PathGeometric &path, int skip)
{
  using namespace boost::numeric::ublas;
  ofstream ofs(filename);
  std::vector<double> angles(Arm.size());
  std::vector<TVector> jpos;
  for (size_t i(0); i < path.getStateCount(); i += skip) {
    const ob::RealVectorStateSpace::StateType *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    for (size_t i(0); i < Arm.size(); ++i) angles[i] = (*s)[i];
    ForwardKinematics(Arm, angles, ArmBase, jpos);
    for (size_t i(0); i < jpos.size(); ++i) ofs << jpos[i] << endl;
    ofs << endl
        << endl;
  }
}


void Planning::CheckArmSequence()
{
  using namespace boost::numeric::ublas;
  std::vector<double> angles(Arm.size());
  std::vector<TVector> jpos;
    // const ob::RealVectorStateSpace::StateType *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    for (size_t i(0); i < Arm.size(); ++i) angles[i] = i;
    ForwardKinematics(Arm, angles, ArmBase, jpos);
    for (size_t i(0); i < jpos.size(); ++i) cout << jpos[i] << endl;
    cout << endl
        << endl;
}


/* Print the planning result into a file.
   The resulting file is a gnuplot script that plots the path,
   the sequence of the arm on the path, and the obstacles.
   Sequence of the arm on ``path'': stored into "res/frame_all.dat",
   obstacles: stored into the resulting script.
   Usage:  gnuplot -persistent filename */
void Planning::PrintArmSolution(const char *filename, const og::PathGeometric &path, int skip)
{
  // ofstream ofs(filename);
  // PrintArmSequence("../plot/frame_all.dat", path, skip);
  // ofs << "\
  // #set terminal png size 800, 640 transparent                     \n\
  // #set terminal svg size 1200 780 fname 'Trebuchet MS' fsize 24   \n\
  // set xlabel 'x'         \n\
  // set ylabel 'y'         \n\
  // set zlabel 'z'         \n\
  // set hidden3d           \n\
  // set ticslevel 0        \n\
  // set size 0.7,1         \n\
  // set parametric         \n\
  // set urange [0:6.28]    \n\
  // set vrange [0:6.28]    \n\
  // set isosample 8,8      \n\
  // set samples 10         \n\
  // r= " << ObstacleRadius << endl;
  // ofs << "splot \\" << endl;
  // for (vector<TVector>::const_iterator itr(Obstacles.begin()), last(Obstacles.end()); itr != last; ++itr) {
  //   const double &ox((*itr)(0)), &oy((*itr)(1)), &oz((*itr)(2));
  //   ofs << "  "
  //       << "r*cos(u)*cos(v)+" << ox << ",r*sin(u)*cos(v)+" << oy << ",r*sin(v)+" << oz
  //       << " w l lt 1 lw 0.2 t '',"
  //       << "\\" << endl;
  // }
  // ofs<<"'frame_all.dat' w lp lt 3 pt 6 lw 1.5"<<endl;
}