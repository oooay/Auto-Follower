/**
 * @file path.cpp
 * @brief Path类：提供跟踪、可视化路径类的函数，主要实现了三个函数：
 *  1) 加入segment: addSegment
 *  2) 加入Node: addNode
 *  3) 加入Vehicle: addVehicle
 *  这三个函数通过updatePath进行整合
 */
#include "path.h"

using namespace HybridAStar;

// ###################################################
//                                          CLEAR PATH
// ###################################################
// 清除路径：将以前的路径位姿点、节点信息、及车辆标记点清除
//          用空值的节点、车辆标记信息对外进行发布
void Path::clear()
{
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  // helon
  // publishPath();
  // publishPathNodes();
  // publishPathVehicles();
}

// ###################################################
//                                          TRACE PATH
// ###################################################
//  __________
//  TRACE PATH
//  对每个3D节点信息，分别进行addSegment、addNode、addVehicle
void Path::updatePath(std::vector<Node3D> nodePath)
{
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i)
  {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}

void Path::updatePathFromPath(nav_msgs::Path inPath)
{
  path.poses = inPath.poses;
}

// ___________
// ADD SEGMENT:将每个node的信息转为geometry_msgs放入向量path中
void Path::addSegment(const Node3D &node)
{
  geometry_msgs::PoseStamped vertex;
  vertex.header.frame_id = Constants::map_frame;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 1;
  // helon
  // path.poses.push_back(vertex);
  path.poses.insert(path.poses.begin(), vertex);
}

// ________
// ADD NODE：节点标记信息放入变量pathNodes
void Path::addNode(const Node3D &node, int i)
{
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0)
  {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = Constants::map_frame;
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed)
  { // 粉红表示平滑的路径点
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  }
  else
  { // 紫色表示没有平滑的路径点
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

/**
 * @brief 将node节点处的车的显示信息加入向量中pathVehicles
 *
 * @param node
 * @param i
 */
void Path::addVehicle(const Node3D &node, int i)
{
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1)
  {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = Constants::map_frame;
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed)
  {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  }
  else
  {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}

// 	helon
void Path::toGlobalPath(tf::TransformListener &listener)
{
  int size = path.poses.size();
  path.poses.clear();
  for (int i = 0; i < size; i++)
  {
    geometry_msgs::PoseStamped vertex;
    listener.waitForTransform(Constants::global_frame, path.header.frame_id, ros::Time(0), ros::Duration(3.0));
    listener.transformPose(Constants::global_frame, path.poses[i], vertex);
    path.poses.push_back(vertex);
  }
  path.header.frame_id = Constants::global_frame;
}

bool Path::isAvailable()
{
  int size = getSize();
  if (size < 5)
  {
    std::cout << "The size of path is " << size << std::endl;
    return false;
  }
  int counter = 0;
  double curvity;
  for (int i = 1; i < size - 1; i++)
  {
    P1 = path.poses[i - 1].pose.position;
    P2 = path.poses[i].pose.position;
    P3 = path.poses[i + 1].pose.position;

    if (P1.y == P2.y && P2.y == P3.y)
    {
      continue; // curvity = 0.0;
    }
    else
    {
      double dis1, dis2, dis3;
      double cosA, sinA, dis;
      dis1 = sqrt((P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y));
      dis2 = sqrt((P1.x - P3.x) * (P1.x - P3.x) + (P1.y - P3.y) * (P1.y - P3.y));
      dis3 = sqrt((P2.x - P3.x) * (P2.x - P3.x) + (P2.y - P3.y) * (P2.y - P3.y));
      dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
      cosA = dis / (2 * dis1 * dis3);    // 余弦定理求角度
      sinA = sqrt(1 - cosA * cosA);      // 求正弦
      curvity = 1 / (0.5 * dis2 / sinA); // 半径的倒数是曲率，半径越小曲率越大
      if (curvity >= 1)
      {
        counter++;
        if (counter > 8)
        {
          std::cout << "The curvity points is more than 5." << std::endl;
          return false;
        }
      }
    }
  }
  return true;
}
