#include "visual_follow_mapping.h"

namespace gmapping 
{
    // 构造函数
GMapping::GMapping() : privatNode("~")
{
    subRegisterCloud = nodeHandle.subscribe("/ground_segmentation/obstacle_cloud", 1, &GMapping::currCloudHandler, this);  //   /cost_PointCloud  
    // map_publisher = privatNode.advertise<nav_msgs::OccupancyGrid>("/occupy_map", 1, true);
    // map_publisher2 = privatNode.advertise<nav_msgs::OccupancyGrid>("/map2", 1, true);
    map_publisher3 = privatNode.advertise<nav_msgs::OccupancyGrid>("/occupy_map", 1, true);
    laserCloudCurr.reset(new pcl::PointCloud<PointType>());

    // 参数初始化
    InitParams();
}

GMapping::~GMapping()
{
    delete gmapping_map;
}

// ros的参数初始化
void GMapping::InitParams()
{
    if (!privatNode.getParam("maxRange", max_range))
        //max_range = range_threshold * 1.2; 
        max_range = 30 - 0.01;
    if (!privatNode.getParam("maxUrange", max_use_range))
        max_use_range = 29; 

    if (!privatNode.getParam("xmin", xmin))
        xmin = -30.0;
    if (!privatNode.getParam("ymin", ymin))
        ymin = -30.0;
    if (!privatNode.getParam("xmax", xmax))
        xmax = 30.0;
    if (!privatNode.getParam("ymax", ymax))
        ymax = 30.0;
    if (!privatNode.getParam("delta", resolution))
        resolution = 0.05;
    if (!privatNode.getParam("occ_thresh", occ_thresh))
        occ_thresh = 0.7;

    /********************************************/
    // 对map_进行初始化
    // 地图的分辨率为0.05m,代表一个格子的距离是0.05m
    map.info.resolution = resolution;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    map.info.width = (xmax - xmin) / map.info.resolution;
    map.info.height = (ymax - ymin) / map.info.resolution;

    // 地图左下角的点对应的物理坐标
    map.info.origin.position.x = xmin;
    map.info.origin.position.y = ymin;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    map.data.resize(map.info.width * map.info.height);

    /********************************************/

    /********************************************/
    // 对map2_进行初始化
    // 地图的分辨率为0.05m,代表一个格子的距离是0.05m
    map2.info.resolution = resolution;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    map2.info.width = 20 / map2.info.resolution;
    map2.info.height = 10 / map2.info.resolution;

    // 地图左下角的点对应的物理坐标
    map2.info.origin.position.x = -5;
    map2.info.origin.position.y = -5;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    map2.data.resize(map2.info.width * map2.info.height);
    /********************************************/


        /********************************************/
    // 对map3_进行初始化
    // 地图的分辨率为0.05m,代表一个格子的距离是0.05m
    map3.info.resolution = resolution;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    map3.info.width = 10 / map3.info.resolution;
    map3.info.height = 20 / map3.info.resolution;

    // 地图左下角的点对应的物理坐标
    map3.info.origin.position.x = -5;
    map3.info.origin.position.y = -5;
    map3.info.origin.orientation.x = 0.70711;
    map3.info.origin.orientation.y = 0.70711;
    map3.info.origin.orientation.z = 0;
    map3.info.origin.orientation.w = 0;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    map3.data.resize(map3.info.width * map3.info.height);
    /********************************************/

    // 地图的中点
    Point center;
    center.x = (xmin + xmax) / 2.0;
    center.y = (ymin + ymax) / 2.0;
    // ScanMatcherMap为GMapping中存储地图的数据类型
    /*****************************************************
    这里的ScanMatcherMap为GMapping中存储地图的数据类型，
    先声明了一个ScanMatcherMap的对象gmapping_map_，
    然后通过ComputeMap()为gmapping_map_赋值，
    然后再将gmapping_map_中存储的值赋值到ros的栅格地图的数据类型中．
    *****************************************************/
    gmapping_map = new ScanMatcherMap( center, xmin, ymin, xmax, ymax, resolution  );
}

//当前帧点云的接收事件函数
void GMapping::currCloudHandler( const sensor_msgs::PointCloud2ConstPtr& msg  )
{
    timeLaserCloudCurr = msg->header.stamp.toSec();
    laserCloudCurr->clear();
    pcl::fromROSMsg( *msg, *laserCloudCurr );
}

void GMapping::run()
{
    {
        map2.header.frame_id = "motor_fixed";
        map.header.frame_id = "motor_fixed";
        map3.header.frame_id = "motor_fixed";

        // 使用当前雷达数据更新GMapping地图中栅格的值
        ComputeMap( gmapping_map, laserCloudCurr );
    }
}

void GMapping::ComputeMap( ScanMatcherMap* gmap, const pcl::PointCloud<PointType>::Ptr currPointCloud)
{
    for (int x = 0; x < map.info.width; ++x)
    {
        for (int y = 0; y < map.info.height; ++y)
        {
            map.data[MAP_IDX(map.info.width, x, y)] = GMAPPING_FREE;
        }
    }

    // 通过激光雷达的数据，找出地图的有效区域
    for( size_t i = 0; i < currPointCloud->points.size(); i++ ) 
    {
        //过滤掉空点
        //if( currPointCloud->points[i].intensity <= 0 )
        //    continue;
        float x1 = currPointCloud->points[i].x;
        float y1 = currPointCloud->points[i].y;
        float z1 = currPointCloud->points[i].z;

        // 排除错误的激光点
        //double d = sqrt( x1 * x1 + y1 * y1 +z1 * z1 );
        double d = sqrt( x1 * x1 + y1 * y1 );
        if (d > max_range || d == 0.0 || !std::isfinite(d))
            continue;
        if (z1 > 0.2 || z1 < -1)
            continue;
        //if( z1 > 1.5 || z1 < -0.5 )
        //    continue;
        // if( x1 > 3 && z1 < -0.35 )
        //     continue;
        // if( x1 > 5 && z1 < -0.28 )
        //     continue;
        
        if( x1 > -1.5 && x1 < 0.1 && abs( y1 ) < 0.36 )
            continue;

        if (d > max_use_range)
            d = max_use_range;

        //点云中的点在世界坐标系下的坐标
        Point phit;
        phit.x = x1;
        phit.y = y1;
        IntPoint p1 = gmap->world2map( phit ); 
        map.data[MAP_IDX(map.info.width, p1.x, p1.y)] = GMAPPING_OCC;
    }
    
    int startX = 25 / map2.info.resolution;
    int startY = 25 / map2.info.resolution;
    for (int y = 0; y < map2.info.height; ++y)
    {
        for (int x = 0; x < map2.info.width; ++x)
        {
            int inputIndex = (startY + y) * map.info.width + (startX + x);
            int outputIndex = y * map2.info.width + x;
            map2.data[outputIndex] = map.data[inputIndex];
        }
    }

    for (int y = 0; y < map2.info.height; ++y)
    {
        for (int x = 0; x < map2.info.width; ++x)
        {
            int inputIndex = y * map2.info.width + x;
            int outputIndex = x * map3.info.width + y;
            map3.data[outputIndex] = map2.data[inputIndex];
        }
    }


    // map.header.stamp = ros::Time().fromSec( timeLaserCloudCurr );
    // map_publisher.publish( map );
    // map2.header.stamp = ros::Time().fromSec( timeLaserOdometry );
    // map_publisher2.publish( map2 );

    map3.header.stamp = ros::Time::now();
    map_publisher3.publish( map3 );
}

} // end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_follow_mapping_node");

    gmapping::GMapping gmap;

    ros::Rate rate( 50 );
    while( ros::ok() )
    {
        ros::spinOnce();        

        gmap.run();

        rate.sleep();
    }

    return (0);
}
