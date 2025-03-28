/**
 * @file planner.cpp
 * @brief 规范器的实现过程
 *
 *
 */

#include "planner.h"

using namespace HybridAStar;
// ###################################################
//                                         CONSTRUCTOR
// ###################################################
Planner::Planner() : _time_num(0)
{
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH：定义了一个发布器
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  pubVelocity = n.advertise<std_msgs::Float32>("/recommend_speed", 1);
  pubCarBorder = n.advertise<geometry_msgs::PolygonStamped>("/car_border", 1);
  pubDetectedBorder = n.advertise<geometry_msgs::PolygonStamped>("/detected_border", 1);

  subMap = n.subscribe("/local_map_occ", 1, &Planner::setMap, this);           // 接收动态地图
  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this); // 接收目标的topic, geometry_msgs/PoseStamped
  // subGoal = n.subscribe("/navigation_goal", 1, &Planner::setGoal, this);
  //  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this); // 接收始点的topic
  // subOdom = n.subscribe("/vehicle/odom", 1, &Planner::odom_cb, this);

  start.header.frame_id = "local_map";
  start.pose.pose.position.x = 5.0;
  start.pose.pose.position.y = 4.6; // default: 5.0
  start.pose.pose.position.z = 0.0;
  start.pose.pose.orientation.x = 0.707107;
  start.pose.pose.orientation.y = 0.707107;
  start.pose.pose.orientation.z = 0.0;
  start.pose.pose.orientation.w = 0.0;
};

// ###################################################
//                                        LOOKUPTABLES
// ###################################################
// 初始化 查找表，主要有两个：Dubins Looup Table及Collision Looup Table
void Planner::initializeLookups()
{
  if (Constants::dubinsLookup)
  {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

// helon
// void Planner::odom_cb(const nav_msgs::Odometry odom_msg)
// {
//   v_current = odom_msg.twist.twist.linear.x;
// }

// ###################################################
//                                                 MAP
// ###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map)
{

  grid = map; // 更新地图指针
  t0 = grid->header.stamp;

  // update the configuration space with the current map
  configurationSpace.updateGrid(map);
  // create array for Voronoi diagram

  int height = map->info.height;
  int width = map->info.width;
  bool **binMap; // 二维数组，
  binMap = new bool *[width];

  for (int x = 0; x < width; x++)
  {
    binMap[x] = new bool[height];
  } // 这里可简化为一次申请

  for (int x = 0; x < width; ++x)
  {
    for (int y = 0; y < height; ++y)
    {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  } // 转化为二值地图

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  // voronoiDiagram.visualize();//将Voronoi Diagram初始化、更新并显示

  // helon
  showBorder();

  validStart = true;
}

// ###################################################
//                                    INITIALIZE START
// ###################################################
// 这是回调函数，当接收到始点时自动调用
/*
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial)
{
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = initial->header.frame_id;
  startN.header.stamp = ros::Time();

  if (startN.header.frame_id != Constants::map_frame)
  {
    // get the start from frame_id to local_map
    // std::cout << "get the start from " << startN.header.frame_id << " to local_map." << std::endl;
    geometry_msgs::PoseStamped startN_localmap;
    try
    {
      listener.transformPose(Constants::map_frame, startN, startN_localmap);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("transfrom exception : %s", ex.what());
      return;
    }
    x = startN_localmap.pose.position.x / Constants::cellSize;
    y = startN_localmap.pose.position.y / Constants::cellSize;
    t = tf::getYaw(startN_localmap.pose.orientation);
    startN.pose.position = startN_localmap.pose.position;
    startN.pose.orientation = startN_localmap.pose.orientation;
    startN.header.frame_id = Constants::map_frame;
  }

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0)
  {
    validStart = true;
    start = *initial;

    // if (Constants::manual) { plan();}//若为静态地图时
    plan();
    // publish start for RViz
    pubStart.publish(startN);
  }
  else
  {
    std::cout << "invalid start x:" << startN.pose.position.x << " y:" << startN.pose.position.y << " t:" << Helper::toDeg(t) << std::endl;
  }
}
*/

// ###################################################
//                                     INITIALIZE GOAL
// ###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr &end)
{

  // oay
  _time_num = 0;

  // retrieving goal position

  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  // float t = tf::getYaw(end->pose.orientation);

  geometry_msgs::PoseStamped goalN;
  goalN.pose.position = end->pose.position;
  goalN.pose.orientation = end->pose.orientation;
  goalN.header.frame_id = end->header.frame_id;
  goalN.header.stamp = t0;

  if (goalN.header.frame_id != Constants::map_frame)
  {
    // get the end from frame_id to local_map
    geometry_msgs::PoseStamped goalN_localmap;
    try
    {
      listener.waitForTransform(Constants::map_frame, goalN.header.frame_id, t0, ros::Duration(3.0));
      listener.transformPose(Constants::map_frame, goalN, goalN_localmap);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("transfrom exception : %s", ex.what());
      return;
    }
    x = goalN_localmap.pose.position.x / Constants::cellSize;
    y = goalN_localmap.pose.position.y / Constants::cellSize;
    // t = tf::getYaw(goalN_localmap.pose.orientation);
    goalN.pose.position = goalN_localmap.pose.position;
    goalN.pose.orientation = goalN_localmap.pose.orientation;
    goalN.header.frame_id = Constants::map_frame;
  }

  // helon (4.6+0.8)/0.1=54, 1/0.1=10
  if (100 >= y && y >= 54 && 90 >= x && x >= 10)
  {
    validGoal = true;
    goal = goalN;
    plan();
  }
  else
  {
    validGoal = false;
    vel.data = 0.0;
    pubVelocity.publish(vel);
  }
}

// ###################################################
//                                       PLAN THE PATH
// ###################################################
void Planner::plan()
{

  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal)
  {

    //  LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D *nodes3D = new Node3D[length]();
    Node2D *nodes2D = new Node2D[width * height]();

    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0, 2PI]
    t = Helper::normalizeHeadingRad(t);
    // std::cout << "The goal x in local_map:" << goal.pose.position.x << " y:" << goal.pose.position.y << " t:" << Helper::toDeg(t) << std::endl;
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    distGoal = abs(start.pose.pose.position.y - goal.pose.position.y);

    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    // std::cout << "The start x in local_map:" << start.pose.pose.position.x << " y:" << start.pose.pose.position.y << " t:" << Helper::toDeg(t) << std::endl;
    Node3D nStart(x, y, t, 0, 0, nullptr);

    ros::Time time0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();

    // 核心步骤：
    //  1) 调用hybridAStar()函数获取一条路径
    //  2) 获取路径点(3D Node) -> 原始路径
    //  3) 对路径依据Voronoi图进行平滑->平滑路径
    //  FIND THE PATH
    Node3D *nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,
                                               configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());

    ros::Time time1 = ros::Time::now();
    ros::Duration d(time1 - time0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // helon
    calcLinear();

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    // 将结果在相应的topic进行发布
    path.publishPath();
    // path.publishPathNodes();
    // path.publishPathVehicles();

    // helon
    if (smoothedPath.isAvailable())
    {
      std::cout << "Path planning succeeded. The length is " << smoothedPath.getSize() << "." << std::endl;
      nav_msgs::Path fittedPath = polynomialFit(smoothedPath.getPath());
      smoothedPath.updatePathFromPath(fittedPath);
      smoothedPath.toGlobalPath(listener);
      smoothedPath.publishPath();
      // smoothedPath.publishPathNodes();
      // smoothedPath.publishPathVehicles();
      // visualization.publishNode3DCosts(nodes3D, width, height, depth);
      // visualization.publishNode2DCosts(nodes2D, width, height);
    }
    else
    {
      std::cout << "The path is not available." << std::endl;
      // _time_num++;
      // if (_time_num > 10) {
      //   smoothedPath.clear();
      //   smoothedPath.publishPath();
      //   vel.data = 0.0;
      //   pubVelocity.publish(vel);
      //   _time_num = 0;
      // }
    };

    delete[] nodes3D;
    delete[] nodes2D;
  }
  else
  {
    std::cout << "missing goal or start." << std::endl;
    path.clear();
    smoothedPath.clear();
    smoothedPath.publishPath();
    // smoothedPath.publishPathNodes();
    // smoothedPath.publishPathVehicles();
    vel.data = 0.0;
    pubVelocity.publish(vel);
  }
}

// oay
void Planner::timeNumAdd()
{
  if (_time_num > 60000)
    _time_num--;
  _time_num++;
}

void Planner::checkTimeNum()
{
  if (_time_num > 10)
  {
    std::cout << "The target is disappeared." << std::endl;
    visualization.clear();
    path.clear();
    smoothedPath.clear();
    smoothedPath.publishPath();
    // smoothedPath.publishPathNodes();
    // smoothedPath.publishPathVehicles();
    vel.data = 0.0;
    pubVelocity.publish(vel);
    _time_num = 0;
  }
}

// helon
bool Planner::isNearObstacle()
{
  // lower_map.info.width=50, lower_map.info.height=100
  // width:3.2m, height:3m, resolution:0.2m/grid
  distObs = distObsMax;
  for (int y = 50; y < 74; y++) // 5.0/0.1, 7.4/0.1
  {
    for (int x = 38; x < 62; x++) // 3.8/0.1, 6.2/0.1
    {
      if (grid->data[x + y * 100])
      {
        double dist = sqrt(pow((x - 50), 2) + pow((y - 46), 2)) * 0.1;
        // std::cout << "detected obstacles, the distance is " << dist << " m." << std::endl;
        if (dist < distObs)
        {
          distObs = dist;
        }
      }
    }
  }
  if (distObs < distObsMax)
  {
    std::cout << "The minimum distObs:" << distObs << " m." << std::endl;
    return true;
  }
  else
  {
    return false;
  }
}

void Planner::calcLinear()
{
  double v = 0.0;
  std::cout << "distGoal:" << distGoal << "." << std::endl;
  v = 1.0 / (1 + pow(10, 2.0 - distGoal));
  if (isNearObstacle())
  {
    v = fminl(-0.1 + distObs / 2.5, v);
  }
  vel.data = fminl(fmaxl(v, 0), 1.2);
  std::cout << "Recommand speed:" << v << "." << std::endl;
  pubVelocity.publish(vel);
}

void Planner::showBorder()
{
  geometry_msgs::Point32 point;
  point.x = Constants::length / 2;
  point.y = Constants::width / 2;
  point.z = 0;
  carBorder.polygon.points.push_back(point);
  point.x = -Constants::length / 2;
  point.y = Constants::width / 2;
  point.z = 0;
  carBorder.polygon.points.push_back(point);
  point.x = -Constants::length / 2;
  point.y = -Constants::width / 2;
  point.z = 0;
  carBorder.polygon.points.push_back(point);
  point.x = Constants::length / 2;
  point.y = -Constants::width / 2;
  point.z = 0;
  carBorder.polygon.points.push_back(point);
  point.x = Constants::length / 2 + 0.5;
  point.y = 0;
  point.z = 0;
  carBorder.polygon.points.push_back(point);

  carBorder.header.frame_id = Constants::base_frame;
  pubCarBorder.publish(carBorder);

  point.x = 0;
  point.y = 1.2;
  point.z = 0;
  detectedBorder.polygon.points.push_back(point);
  point.x = 2.4;
  point.y = 1.2;
  point.z = 0;
  detectedBorder.polygon.points.push_back(point);
  point.x = 2.4;
  point.y = -1.2;
  point.z = 0;
  detectedBorder.polygon.points.push_back(point);
  point.x = 0;
  point.y = -1.2;
  point.z = 0;

  detectedBorder.polygon.points.push_back(point);
  detectedBorder.header.frame_id = Constants::base_frame;
  pubDetectedBorder.publish(detectedBorder);
}

nav_msgs::Path Planner::polynomialFit(const nav_msgs::Path newPath)
{
  if (newPath.poses.size() > 0)
  {
    paths.push_back(newPath);
  }
  if (paths.size() > 5)
  {
    paths.erase(paths.begin());
  }

  std::cout << "paths.size(): " << paths.size() << std::endl;

  int maxSize = 0;
  int counter = 0;

  for (const auto &path : paths)
  {
    maxSize = std::max(maxSize, static_cast<int>(path.poses.size()));
  }

  std::cout << "maxSize: " << maxSize << std::endl;

  if (paths.size() == 0)
  {
    return newPath;
  }

  nav_msgs::Path fittedPath;
  fittedPath.header.frame_id = Constants::map_frame;
  fittedPath.header.stamp = ros::Time(0);
  for (unsigned int i = 0; int(i) < maxSize; i++)
  {
    double addX = 0.0;
    double addY = 0.0;
    counter = 0;
    for (unsigned int j = 0; j < paths.size(); j++)
    {
      if (i < paths[j].poses.size())
      {
        addX += paths[j].poses[i].pose.position.x;
        addY += paths[j].poses[i].pose.position.y;
        counter++;
      }
    }
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = Constants::map_frame;
    pose.pose.position.x = addX / counter;
    pose.pose.position.y = addY / counter;
    pose.pose.orientation.w = 1.0;
    fittedPath.poses.push_back(pose);
  }

  std::cout << "fittedPath.size: " << fittedPath.poses.size() << std::endl;
  std::cout << "-----------------------------" << std::endl;

  return fittedPath;
}
