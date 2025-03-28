/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
   这是ROS的主函数入口程序：即启动规划器入口
*/

// ###################################################
//                       HYBRID A* ALGORITHM
//   AUTHOR:   Karl Kurzer
//   WRITTEN:  2015-03-02
// ###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "unistd.h"

#include "constants.h"
#include "planner.h"

// ###################################################
//                               COUT STANDARD MESSAGE
// ###################################################
/**
   \fn message(const T& msg, T1 val = T1())
   \brief Convenience method to display text
*/
template <typename T, typename T1>
void message(const T &msg, T1 val = T1())
{
  if (!val)
  {
    std::cout << "### " << msg << std::endl;
  }
  else
  {
    std::cout << "### " << msg << val << std::endl;
  }
}

// ###################################################
//                                                MAIN
// ###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char **argv)
{

  sleep(2.0);

  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual)
  {
    message("mode: ", "manual"); // 静态地图
  }
  else
  {
    message("mode: ", "auto"); // 动态地图
  }

  ros::init(argc, argv, "hybrid_astar");

  HybridAStar::Planner hy;
  hy.plan();

  // oay
  ros::Rate rate(10); // default:10hz

  while (ros::ok())
  {
    ros::spinOnce();
    hy.timeNumAdd();
    hy.checkTimeNum();
    rate.sleep();
  }

  // ros::spin();
  return 0;
}
