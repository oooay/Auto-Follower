/**
 * @file node3d.cpp
 * @brief 实现了前向行驶和逆向行驶的代价计算
 *
 */
#include "node3d.h"

using namespace HybridAStar;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3; // 三个方向。注意2D Node为8个
// possible movements
// const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
// const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
// const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

// R = 6, 6.75 DEG: 当R=6时，每个方向的变化量
// const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
// const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
// const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

// const float Node3D::dy[] = { 0,       -0.16578,     0.16578};
// const float Node3D::dx[] = { 1.41372,  1.40067,     1.40067};
// const float Node3D::dt[] = { 0,        0.23562,    -0.23562};

// R = 3.6
// const float Node3D::dy[] = { 0,          -0.0249536,    0.0249536};
// const float Node3D::dx[] = { 0.4238699,   0.4231347,    0.4231347};
// const float Node3D::dt[] = { 0,           0.1178097,   -0.1178097};

// R = 4.0, 6.75 DEG
// const float Node3D::dy[] = { 0,          -0.0277262,    0.0277262};
// const float Node3D::dx[] = { 0.4709664,   0.4701496,    0.4701496};
// const float Node3D::dt[] = { 0,           0.1178097,   -0.1178097};

// R = 4.0, 10 DEG
const float Node3D::dy[] = {0, -0.0608, 0.0608};
const float Node3D::dx[] = {0.6972, 0.6946, 0.6946};
const float Node3D::dt[] = {0, 0.1745, -0.1745};

// ###################################################
//                                          IS ON GRID
// ###################################################
// 判断是否在3D网格上， x \in [0, width], y \in [0, height], t/deltaHeadingRad \in [0, headings]
bool Node3D::isOnGrid(const int width, const int height) const
{
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}

// ###################################################
//                                         IS IN RANGE
// ###################################################
bool Node3D::isInRange(const Node3D &goal) const
{
  int random = rand() % 10 + 1; // 产生位于[1, 10]的随机数
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance; // 距离的平方和在100以内则认为可达
}

// ###################################################
//                                    CREATE SUCCESSOR
//  产生Successor
// ###################################################
Node3D *Node3D::createSuccessor(const int i)
{
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor p ositions forward
  if (i < 3)
  { // 前向 Successor
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else
  { // 后向 Successor
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

// ###################################################
//                                       MOVEMENT COST
// ###################################################
// 更新代价函数：cost-so-far
void Node3D::updateG()
{
  // forward driving
  if (prim < 3)
  { // 前进情况
    // penalize turning
    if (pred->prim != prim)
    { // 方向发生改变时
      // penalize change of direction
      if (pred->prim > 2)
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD; // 改变方向的惩罚
      }
      else
      {
        g += dx[0] * Constants::penaltyTurning; // 没有改变方向
      }
    }
    else
    { // 方向没有发生变化
      g += dx[0];
    }
  }
  // reverse driving
  else
  { // 后退
    // penalize turning and reversing
    if (pred->prim != prim)
    {
      // penalize change of direction
      if (pred->prim < 3)
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      }
      else
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    }
    else
    {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}

////###################################################
////                                         COST TO GO
////###################################################
// void Node3D::updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, Node2D* nodes2D, float* dubinsLookup, Visualize& visualization) {
//   float dubinsCost = 0;
//   float reedsSheppCost = 0;
//   float twoDCost = 0;
//   float twoDoffset = 0;

//  // if dubins heuristic is activated calculate the shortest path
//  // constrained without obstacles
//  if (Constants::dubins) {

//// ONLY FOR dubinsLookup
////    int uX = std::abs((int)goal.x - (int)x);
////    int uY = std::abs((int)goal.y - (int)y);
////    // if the lookup table flag is set and the vehicle is in the lookup area
////    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
////      int X = (int)goal.x - (int)x;
////      int Y = (int)goal.y - (int)y;
////      int h0;
////      int h1;

////      // mirror on x axis
////      if (X >= 0 && Y <= 0) {
////        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
////        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.t) / Constants::deltaHeadingRad);
////      }
////      // mirror on y axis
////      else if (X <= 0 && Y >= 0) {
////        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
////        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.t) / Constants::deltaHeadingRad);

////      }
////      // mirror on xy axis
////      else if (X <= 0 && Y <= 0) {
////        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
////        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.t) / Constants::deltaHeadingRad);

////      } else {
////        h0 = (int)(t / Constants::deltaHeadingRad);
////        h1 = (int)(goal.t / Constants::deltaHeadingRad);
////      }

////      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
////                                + uY *  Constants::headings * Constants::headings
////                                + h0 * Constants::headings
////                                + h1];
////    } else {

//        /*if (Constants::dubinsShot && std::abs(x - goal.x) >= 10 && std::abs(y - goal.y) >= 10)*/
////      // start
////      double q0[] = { x, y, t};
////      // goal
////      double q1[] = { goal.x, goal.y, goal.t};
////      DubinsPath dubinsPath;
////      dubins_init(q0, q1, Constants::r, &dubinsPath);
////      dubinsCost = dubins_path_length(&dubinsPath);

//      ompl::base::DubinsStateSpace dubinsPath(Constants::r);
//      State* dbStart = (State*)dubinsPath.allocState();
//      State* dbEnd = (State*)dubinsPath.allocState();
//      dbStart->setXY(x, y);
//      dbStart->setYaw(t);
//      dbEnd->setXY(goal.x, goal.y);
//      dbEnd->setYaw(goal.t);
//      dubinsCost = dubinsPath.distance(dbStart, dbEnd);
//  }

//  // if reversing is active use a
//  if (Constants::reverse && !Constants::dubins) {
//    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
//    State* rsStart = (State*)reedsSheppPath.allocState();
//    State* rsEnd = (State*)reedsSheppPath.allocState();
//    rsStart->setXY(x, y);
//    rsStart->setYaw(t);
//    rsEnd->setXY(goal.x, goal.y);
//    rsEnd->setYaw(goal.t);
//    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
//  }

//  // if twoD heuristic is activated determine shortest path
//  // unconstrained with obstacles
//  if (Constants::twoD && !nodes2D[(int)y * grid->info.width + (int)x].isDiscovered()) {
//    // create a 2d start node
//    Node2D start2d(x, y, 0, 0, nullptr);
//    // create a 2d goal node
//    Node2D goal2d(goal.x, goal.y, 0, 0, nullptr);
//    // run 2d astar and return the cost of the cheapest path for that node
//    nodes2D[(int)y * grid->info.width + (int)x].setG(Algorithm::aStar(goal2d, start2d, grid, nodes2D, visualization));
//  }

//  if (Constants::twoD) {
//    // offset for same node in cell
//    twoDoffset = sqrt(((x - (long)x) - (goal.x - (long)goal.x)) * ((x - (long)x) - (goal.x - (long)goal.x)) +
//                      ((y - (long)y) - (goal.y - (long)goal.y)) * ((y - (long)y) - (goal.y - (long)goal.y)));
//    twoDCost = nodes2D[(int)y * grid->info.width + (int)x].getG() - twoDoffset;

//  }

//  // return the maximum of the heuristics, making the heuristic admissable
//  h = std::max(reedsSheppCost, std::max(dubinsCost, twoDCost));
//}

////###################################################
////                                 COLLISION CHECKING
////###################################################
// bool Node3D::isTraversable(const nav_msgs::OccupancyGrid::ConstPtr& grid, Constants::config* collisionLookup) const {
//   int X = (int)x;
//   int Y = (int)y;
//   int iX = (int)((x - (long)x) * Constants::positionResolution);
//   iX = iX > 0 ? iX : 0;
//   int iY = (int)((y - (long)y) * Constants::positionResolution);
//   iY = iY > 0 ? iY : 0;
//   int iT = (int)(t / Constants::deltaHeadingRad);
//   int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
//   int cX;
//   int cY;

//  for (int i = 0; i < collisionLookup[idx].length; ++i) {
//    cX = (X + collisionLookup[idx].pos[i].x);
//    cY = (Y + collisionLookup[idx].pos[i].y);

//    // make sure the configuration coordinates are actually on the grid
//    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
//      if (grid->data[cY * grid->info.width + cX]) {
//        return false;
//      }
//    }
//  }

//  return true;
//}

////###################################################
////                                        DUBINS SHOT
////###################################################
// Node3D* Node3D::dubinsShot(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, Constants::config* collisionLookup) const {
//   // start
//   double q0[] = { x, y, t };
//   // goal
//   double q1[] = { goal.x, goal.y, goal.t };
//   // initialize the path
//   DubinsPath path;
//   // calculate the path
//   dubins_init(q0, q1, Constants::r, &path);

//  int i = 0;
//  float x = 0.f;
//  float length = dubins_path_length(&path);

//  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

//  while (x <  length) {
//    double q[3];
//    dubins_path_sample(&path, x, q);
//    dubinsNodes[i].setX(q[0]);
//    dubinsNodes[i].setY(q[1]);
//    dubinsNodes[i].setT(helper::normalizeHeadingRad(q[2]));

//    // collision check
//    if (dubinsNodes[i].isTraversable(grid, collisionLookup)) {

//      // set the predecessor to the previous step
//      if (i > 0) {
//        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
//      } else {
//        dubinsNodes[i].setPred(this);
//      }

//      if (&dubinsNodes[i] == dubinsNodes[i].pred) {
//        std::cout << "looping shot";
//      }

//      x += Constants::dubinsStepSize;
//      i++;
//    } else {
//      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
//      // delete all nodes
//      delete [] dubinsNodes;
//      return nullptr;
//    }
//  }

//  //  std::cout << "Dubins shot connected, returning the path" << "\n";
//  return &dubinsNodes[i - 1];
//}

// ###################################################
//                                  3D NODE COMPARISON
// ###################################################
// 3d节点的比较函数：x和y同时相同、并且theta在一个阈值范围内时可以认为是同一个Node
bool Node3D::operator==(const Node3D &rhs) const
{
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
