#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "constants.h"

using namespace HybridAStar;

nav_msgs::OccupancyGridPtr grid;
nav_msgs::OccupancyGrid lower_map;

ros::Time time1;

void scaleUpMap(const nav_msgs::OccupancyGrid::Ptr map)
{

  // bool isBreakLoop = false;
  // lower_map.header.stamp = ros::Time::now();
  // lower_map.header.frame_id = Constants::map_frame;
  // lower_map.info.resolution = map->info.resolution * 4;
  // lower_map.info.width = map->info.width / 4;   // 50
  // lower_map.info.height = map->info.height / 4; // 100
  // lower_map.info.origin.position.x = 0.0;
  // lower_map.info.origin.position.y = 0.0;
  // lower_map.info.origin.position.z = 0.0;
  // lower_map.info.origin.orientation.x = 0.0;
  // lower_map.info.origin.orientation.y = 0.0;
  // lower_map.info.origin.orientation.z = 0.0;
  // lower_map.info.origin.orientation.w = 1.0;
  // lower_map.data.clear();

  // for (unsigned int h = 0; h < map->info.height; h += 4)
  // {
  //   for (unsigned int w = 0; w < map->info.width; w += 4)
  //   {
  //     isBreakLoop = false;
  //     for (int i = 0; i < 4 && !isBreakLoop; i++)
  //     {
  //       for (int j = 0; j < 4 && !isBreakLoop; j++)
  //       {
  //         if (map->data[(w + j) + (h + i) * map->info.width])
  //         {
  //           lower_map.data.push_back(100);
  //           isBreakLoop = true;
  //         }
  //       }
  //     }
  //     if (!isBreakLoop)
  //     {
  //       lower_map.data.push_back(0);
  //     }
  //   }
  // }

  bool isBreakLoop = false;
  lower_map.header.stamp = ros::Time::now();
  lower_map.header.frame_id = Constants::map_frame;
  lower_map.info.resolution = map->info.resolution * 2;
  lower_map.info.width = map->info.width / 2;   // 100
  lower_map.info.height = map->info.height / 2; // 200
  lower_map.info.origin.position.x = 0.0;
  lower_map.info.origin.position.y = 0.0;
  lower_map.info.origin.position.z = 0.0;
  lower_map.info.origin.orientation.x = 0.0;
  lower_map.info.origin.orientation.y = 0.0;
  lower_map.info.origin.orientation.z = 0.0;
  lower_map.info.origin.orientation.w = 1.0;
  lower_map.data.clear();

  for (unsigned int h = 0; h < map->info.height; h += 2)
  {
    for (unsigned int w = 0; w < map->info.width; w += 2)
    {
      isBreakLoop = false;
      for (int i = 0; i < 2 && !isBreakLoop; i++)
      {
        for (int j = 0; j < 2 && !isBreakLoop; j++)
        {
          if (map->data[(w + j) + (h + i) * map->info.width])
          {
            lower_map.data.push_back(100);
            isBreakLoop = true;
          }
        }
      }
      if (!isBreakLoop)
      {
        lower_map.data.push_back(0);
      }
    }
  }
}

void setMap(const nav_msgs::OccupancyGrid::Ptr map)
{
  grid = map;
  time1 = map->header.stamp;
  scaleUpMap(grid);
}

int main(int argc, char **argv)
{
  // initiate the broadcaster
  ros::init(argc, argv, "scale_up_map");
  ros::NodeHandle nh;

  tf::TransformBroadcaster broadcaster;
  tf::Pose tfPose;

  // subscribe to map updates
  ros::Subscriber sub_map = nh.subscribe("/occupy_map", 1, setMap);
  ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/local_map_occ", 1);

  ros::Rate r(10);

  while (ros::ok())
  {

    // broadcaster.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(tf::Quaternion(0.707107, 0.707107, 0, 0), tf::Vector3(-4.5, -5, -0.42)),
    //     ros::Time::now(), "base_link", Constants::map_frame));

    pub_map.publish(lower_map);

    ros::spinOnce();

    r.sleep();
  }
}
