#include "visual_follow_mapping.h"

namespace gmapping 
{

GMapping::GMapping() : privatNode("~")
{
    subRegisterCloud = nodeHandle.subscribe("/ground_segmentation/obstacle_cloud", 1, &GMapping::currCloudHandler, this);  //   /cost_PointCloud  
    map_publisher3 = privatNode.advertise<nav_msgs::OccupancyGrid>("/occupy_map", 1, true);
    laserCloudCurr.reset(new pcl::PointCloud<PointType>());

    // Parameter initialization
    InitParams();
}

GMapping::~GMapping()
{
    delete gmapping_map;
}

// Initialization of parameters for ros
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
    // Initialize map_
    // The resolution of the map is 0.05m, which means that the distance of a grid is 0.05m.
    map.info.resolution = resolution;

    // Map image size in pixels, width is how many pixels the width of the map is
    map.info.width = (xmax - xmin) / map.info.resolution;
    map.info.height = (ymax - ymin) / map.info.resolution;

    // Physical coordinates corresponding to the points in the lower left corner of the map
    map.info.origin.position.x = xmin;
    map.info.origin.position.y = ymin;

    // Initialize the array with the actual number of pixels.
    map.data.resize(map.info.width * map.info.height);

    /********************************************/

    /********************************************/
    // Initialize map2_
    // The resolution of the map is 0.05m, which means that the distance of a grid is 0.05m.
    map2.info.resolution = resolution;

    // Map image size in pixels, width is how many pixels the width of the map is
    map2.info.width = 20 / map2.info.resolution;
    map2.info.height = 10 / map2.info.resolution;

    // Physical coordinates corresponding to the points in the lower left corner of the map
    map2.info.origin.position.x = -5;
    map2.info.origin.position.y = -5;

    // Initialize the array with the actual number of pixels.
    map2.data.resize(map2.info.width * map2.info.height);
    /********************************************/


        /********************************************/
    // Initialize map3_
    // The resolution of the map is 0.05m, which means that the distance of a grid is 0.05m.
    map3.info.resolution = resolution;

    // Map image size in pixels, width is how many pixels the width of the map is
    map3.info.width = 10 / map3.info.resolution;
    map3.info.height = 20 / map3.info.resolution;

    // Physical coordinates corresponding to the points in the lower left corner of the map
    map3.info.origin.position.x = -5;
    map3.info.origin.position.y = -5;
    map3.info.origin.orientation.x = 0.70711;
    map3.info.origin.orientation.y = 0.70711;
    map3.info.origin.orientation.z = 0;
    map3.info.origin.orientation.w = 0;

    // Initialize the array with the actual number of pixels.
    map3.data.resize(map3.info.width * map3.info.height);
    /********************************************/

    // Midpoint of the map
    Point center;
    center.x = (xmin + xmax) / 2.0;
    center.y = (ymin + ymax) / 2.0;
    // ScanMatcherMap is the data type for storing maps in GMapping
    /*****************************************************
    Here ScanMatcherMap is the data type of the map stored in GMapping.
    First declare a ScanMatcherMap object gmapping_map_，
    Then assign a value to gmapping_map_ via ComputeMap()，
    The value stored in gmapping_map_ is then assigned to the data type of ros's raster map．
    *****************************************************/
    gmapping_map = new ScanMatcherMap( center, xmin, ymin, xmax, ymax, resolution  );
}

//Receive event function for the current frame of the point cloud
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

        // Updating raster values in GMapping maps with current radar data
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

    // Find out the effective area of the map by using the data from the LiDAR
    for( size_t i = 0; i < currPointCloud->points.size(); i++ ) 
    {
        float x1 = currPointCloud->points[i].x;
        float y1 = currPointCloud->points[i].y;
        float z1 = currPointCloud->points[i].z;

        // Troubleshooting erroneous laser points
        double d = sqrt( x1 * x1 + y1 * y1 );
        if (d > max_range || d == 0.0 || !std::isfinite(d))
            continue;
        if (z1 > 0.2 || z1 < -1)
            continue;
        
        if( x1 > -1.5 && x1 < 0.1 && abs( y1 ) < 0.36 )
            continue;

        if (d > max_use_range)
            d = max_use_range;

        //Coordinates of the points in the point cloud in the world coordinate system
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

    map3.header.stamp = ros::Time::now();
    map_publisher3.publish( map3 );
}

}

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
