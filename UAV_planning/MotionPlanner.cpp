#include "MotionPlanner.h"
#include "p3-uav-helper.h"
using namespace std;

// コンストラクタ
Planning::Planning(std::string fileName):
  xLeft(0),
  xRight(6),
  yTop(5),
  yBottom(0),
  zTop(5),
  zBottom(0)
{
  CreateMap();
  PlannerSelector();
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


/* Generate ``num'' shperes and store them into ``Obstacles''
    where each center is decided randomly. */
void Planning::CreateMap()
{
  srand((unsigned int)time(NULL));
  Obstacles.resize(num);
  for (int i(0); i < num; ++i) {
    Obstacles[i].resize(3);
    Obstacles[i](0) = SizeX * (double)rand() / (double)RAND_MAX;
    Obstacles[i](1) = SizeY * (double)rand() / (double)RAND_MAX;
    Obstacles[i](2) = SizeZ * (double)rand() / (double)RAND_MAX;
  }
}


bool Planning::isStateValid(const ob::State *state)
{
  const ob::SE3StateSpace::StateType *state_3d= state->as<ob::SE3StateSpace::StateType>();
  const double &x(state_3d->getX()), &y(state_3d->getY()), &z(state_3d->getZ());
  double length;

  for (int i = 0; i < num; ++i){
    length = pow(x-Obstacles[i](0),2)+pow(y-Obstacles[i](1),2)+pow(z-Obstacles[i](2),2);
    if (sqrt(length) < RobotRadius+ObstacleRadius){
      return false;
    }
  }

  return true;
}


// Print a vertex to file
void Planning::printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex)
{
  std::vector<double> reals;
  if(vertex!=ob::PlannerData::NO_VERTEX)// 頂点が存在しない状態じゃなかったら
  {
    space->copyToReals(reals, vertex.getState());// Copy all the real values from a state source to the array reals using getValueAddressAtLocation()
    for(size_t j = 0; j < reals.size(); ++j){
      os << " " << reals[j];
    }
  }
}


void Planning::planWithSimpleSetup()
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE3StateSpace());

  ob::RealVectorBounds bounds(3);
  bounds.setLow(0,xLeft);
  bounds.setHigh(0,xRight);
  bounds.setLow(1,yBottom);
  bounds.setHigh(1,yTop);
  bounds.setLow(2,zBottom);
  bounds.setHigh(2,zTop);
  space->as<ob::SE3StateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&Planning::isStateValid, this, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::SE3StateSpace> start(space);
  // start->setXYZ(xStart,yStart,zStart);
  // start->rotation().setIdentity();
  start.random();
  cout << "start: "; start.print(cout);

  ob::ScopedState<ob::SE3StateSpace> goal(space);
  // goal->setXYZ(xGoal,yGoal,zGoal);
  // goal->rotation().setIdentity();
  goal.random();
  cout << "goal: "; goal.print(cout);

  ss.setStartAndGoalStates(start, goal);

  if(selector == 1){
    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 2){
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 3){
    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 4){
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 5){
    ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 6){
    ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 7){
    ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 8){
    ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 9){
    ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }

  cout << "----------------" << endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(10);

  // If we have a solution,
  if (solved)
  {
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
    // PrintBoxSequence("../plot/UAV.dat", ss.getSolutionPath());
    PrintSolution("../plot/UAV.plt", ss.getSolutionPath());

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss.getSpaceInformation());
    ss.getPlannerData(pdat);

    // Print the vertices to file
    std::ofstream ofs_v("../plot/vertices.dat");
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
      printEdge(ofs_v, ss.getStateSpace(), pdat.getVertex(i));
      ofs_v<<endl;
    }

    // Print the edges to file
    std::ofstream ofs_e("../plot/edges.dat");
    std::vector<unsigned int> edge_list;
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
      unsigned int n_edge= pdat.getEdges(i,edge_list);
      for(unsigned int i2(0); i2<n_edge; ++i2)
      {
        printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(i));
        ofs_e<<endl;
        printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(edge_list[i2]));
        ofs_e<<endl;
        ofs_e<<endl<<endl;
      }
    }
  }
  else
    cout << "No solution found" << endl;
}


/* Print every center shperes into a file "res/map.dat". */
void Planning::PrintMap()
{
  using namespace std;
  ofstream ofs("../plot/map.dat");
  for (vector<TVector>::const_iterator itr(Obstacles.begin()), last(Obstacles.end()); itr != last;
       ++itr)
    ofs << (*itr) << endl;
}

/* Save a sequence of box on ``path'' into file that is gnuplot-compatible.
    The path should be a sequence of SE(3) state. The box size is ``(sizex,sizey,sizez)''.
    The parameter ``skip'' is an interval to sample from ``path'' (1 for every sample). */
void Planning::PrintBoxSequence(const char *filename, const og::PathGeometric &path, int skip)
{
  using namespace std;
  using namespace boost::numeric::ublas;
  ofstream ofs(filename);
  for (size_t i(0); i < path.getStateCount(); i += skip) {
    const ob::SE3StateSpace::StateType *s = path.getState(i)->as<ob::SE3StateSpace::StateType>();
    TVector pos(3), d(3);
    TMatrix R = QtoR(s->rotation());
    pos(0) = s->getX();
    pos(1) = s->getY();
    pos(2) = s->getZ();
    ofs << pos + prod(R, V3(RobotX, RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, -RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, -RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, -RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, -RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, -RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, -RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, -RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, -RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, RobotY, RobotZ)) << endl;
    ofs << pos + prod(R, V3(-RobotX, RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, RobotY, -RobotZ)) << endl;
    ofs << pos + prod(R, V3(RobotX, RobotY, RobotZ)) << endl;
    ofs << endl
        << endl;
  }
}


/* Print the planning result into a file.
    The resulting file is a gnuplot script that plots the path,
    the sequence of box on the path, and the obstacles.
    ``path'': stored into "res/path.dat",
    sequence of box on ``path'': stored into "res/frame_all.dat",
    obstacles: stored into the resulting script.
    Usage:  gnuplot -persistent filename */
void Planning::PrintSolution(const char *filename, const og::PathGeometric &path, int skip)
{
  using namespace std;
  ofstream ofs(filename);
  {
    ofstream ofs("../plot/path.dat");
    path.printAsMatrix(ofs);
  }
  PrintBoxSequence("../plot/frame_all.dat", path, skip);
  ofs << "\
          #set terminal png size 800, 640 transparent                     \n\
          #set terminal svg size 1200 780 fname 'Trebuchet MS' fsize 24   \n\
          set xlabel 'x'         \n\
          set ylabel 'y'         \n\
          set zlabel 'z'         \n\
          set hidden3d           \n\
          set ticslevel 0        \n\
          set size 0.7,1         \n\
          set parametric         \n\
          set urange [0:6.28]    \n\
          set vrange [0:6.28]    \n\
          set isosample 8,8      \n\
          set samples 10         \n\
          r= "
      << ObstacleRadius << endl;
  ofs << "splot \\" << endl;
  for (vector<TVector>::const_iterator itr(Obstacles.begin()), last(Obstacles.end()); itr != last;
       ++itr) {
    const double &ox((*itr)(0)), &oy((*itr)(1)), &oz((*itr)(2));
    ofs << "  "
        << "r*cos(u)*cos(v)+" << ox << ",r*sin(u)*cos(v)+" << oy << ",r*sin(v)+" << oz
        << " w l lt 1 lw 0.2 t '',"
        << "\\" << endl;
  }
  ofs << "'frame_all.dat' w l lt 3, \\" << endl;
  ofs << "'path.dat' w l lt 4" << endl;
}


// 参考：http://www-sens.sys.es.osaka-u.ac.jp/wakate/tutorial/group3/gnuplot/
int Planning::OpenGnuplot()
{
  // output_plt("../plot/plot.plt");

  FILE *fp = popen("cd ../plot && gnuplot -persist", "w");
  if (fp == NULL){
    return -1;
  }
  fputs("set mouse\n", fp);
  fputs("load \"plot.plt\"\n", fp);

  fflush(fp);
  cin.get();
  pclose(fp);
  return 0;
}