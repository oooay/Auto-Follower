#ifndef PUB_OPTIMAL_GOAL_H
#define PUB_OPTIMAL_GOAL_H

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include "constants.h"

namespace HybridAStar
{

  class OptimalGoal
  {
  public:
    OptimalGoal();

    void setMap(const nav_msgs::OccupancyGrid::Ptr map);

    void setGoal(const geometry_msgs::PoseStamped goal_msg);

    void calcCrossPointsOfTwoCircles(double x1, double y1, double r1, double x2, double y2, double r2);

    bool isNearObstacle(int x, int y);

    void showBorder(double x, double y);

  private:
    ros::NodeHandle nh;
    ros::Subscriber subMap;
    ros::Subscriber subGoal;
    ros::Publisher pubGoal;
    ros::Publisher pubDetectedBorder;
    tf::TransformListener listener;
    ros::Time time1;
    nav_msgs::OccupancyGridPtr grid;
    geometry_msgs::PoseStamped optimal_goal;
    geometry_msgs::PolygonStamped detectedBorder;
    geometry_msgs::Quaternion q;

    double goal_time;
    bool goal_updated;
    double x0, y0, x1, y1, r1, x2, y2, r2, x3, y3, x4, y4;
    int cx1, cy1, cx3, cy3, cx4, cy4;
  };
}

#endif
