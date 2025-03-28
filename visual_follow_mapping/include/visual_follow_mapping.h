#ifndef __VISUAL_FOLLOW_MAPPING
#define __VISUAL_FOLLOW_MAPPING

#include <ros/ros.h>

#include "map.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <nav_msgs/OccupancyGrid.h>

namespace gmapping 
{

    #define GMAPPING_UNKNOWN (-1)
    #define GMAPPING_FREE (0)
    #define GMAPPING_OCC (100)

    typedef pcl::PointXYZI  PointType;

    //从smap的二位数组存储格式，转到一维数组，数组序号也需要转换到一维数组
    #define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class GMapping
{
    public:
    GMapping();
    ~GMapping();
    void InitParams();
    void ComputeMap(ScanMatcherMap* gmap, const pcl::PointCloud<PointType>::Ptr currPointCloud);
    void currCloudHandler( const sensor_msgs::PointCloud2ConstPtr& msg  ); // 当前帧点云的接收事件函数
    void run();

    private:
    ros::NodeHandle nodeHandle;           // ros中的句柄
    ros::NodeHandle privatNode;          // ros中的私有句柄
    ros::Subscriber subRegisterCloud; // 声明一个Subscriber,接收当前帧点云话题

    ros::Publisher map_publisher;          // 声明一个Publisher
    ros::Publisher map_publisher2;
    ros::Publisher map_publisher3;

    nav_msgs::OccupancyGrid map;           //用来发布map的实体对象
    nav_msgs::OccupancyGrid map2;          //cai jian hou de di tu
    nav_msgs::OccupancyGrid map3;
    
    double max_range;      // 激光雷达最大的量程
    double max_use_range;  // 激光雷达最大使用距离

    double xmin;           // 地图的边界,m为单位
    double ymin;
    double xmax;
    double ymax;
    double resolution;     // 地图的分辨率，m为单位
    double occ_thresh;     // 大于这个阈值的格子才认为是占用栅格
    double timeLaserCloudCurr; //当前帧点云的时间信息

    float range_threshold = 1.2; //建图点半径的阈值

    pcl::PointCloud<PointType>::Ptr laserCloudCurr; //当前帧下的点云数据

    ScanMatcherMap* gmapping_map; //创建ScanMatcherMap对象
};

} // end namespace
#endif 