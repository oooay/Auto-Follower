#include "target_localization.h"


TargetLocalization::TargetLocalization( ros::NodeHandle &nh, ros::NodeHandle &private_nh ):
nh_( nh ), 
private_nh_( private_nh ), 
n( nh ), 
nav_goal_update_( false ),
point_image_init_( false )
{
    //param init
    paramInit();

    //points_image_pub_ = n.advertise("/points_image", 1);
    image_pointscloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("image_pointcloud", 1);
    image_points_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("image_points", 1);
    goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>( "navigation_goal", 5 );
    target_localization_pub = private_nh_.advertise<geometry_msgs::PoseStamped>( "target_localization", 5 );
    target_motor_position_pub_ = private_nh_.advertise<geometry_msgs::Point> ("target_motor_position", 1);
    detect_result_sub_ = nh_.subscribe( detect_box_topic_, 1, &TargetLocalization::detectResultCallback, this );
    nav_goal_thread_ = new std::thread( TargetLocalization::naveGoalPub, this );

    //Synchronized data from lidar and camera
    rgb_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>( nh_, image_topic_, 1 );
    lidar_points_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( nh_, pointcloud_topic_, 1 );
    sync_1_ = new message_filters::Synchronizer<sync_pol_1_>( sync_pol_1_(10), *rgb_image_sub_, *lidar_points_sub_ );
    sync_1_->registerCallback( boost::bind( &TargetLocalization::sensorMessageCallBack, this, _1, _2 ) );

    //Initialization of transform between camera and lidar
    intrinsicExtrinsicInit();
}

TargetLocalization::~TargetLocalization()
{
    delete nav_goal_thread_;
    delete rgb_image_sub_;
    delete lidar_points_sub_;
    delete sync_1_;
}

void TargetLocalization::paramInit()
{
    private_nh_.param<std::string>( "pointCloudTopic", pointcloud_topic_, "/velodyne_points" );
    private_nh_.param<std::string>( "imageTopic", image_topic_, "/image" );
    private_nh_.param<std::string>( "detectBoxTopic_", detect_box_topic_, "/vision_detector/box_result" );
    private_nh_.param<std::vector<double>>("extrinsicRPY", ext_rot_, std::vector<double>());
    private_nh_.param<std::vector<double>>("extrinsicTrans", ext_trans_, std::vector<double>());
    private_nh_.param<std::vector<double>>("intransicCamera", camera_param_, std::vector<double>());
    private_nh_.param<double>("distanceThreshold", distance_threshold_, 1.5);
}

void TargetLocalization::intrinsicExtrinsicInit()
{
    lidar_camera_trans_ = Eigen::Matrix4f::Identity();
    camera_lidar_trans_ = Eigen::Matrix4f::Identity();

    Eigen::Vector3d lidar_camera_rpy( ext_rot_[0], ext_rot_[1], ext_rot_[2] );
    Eigen::Matrix3d lidar_camera_rotation;
    lidar_camera_rotation = Eigen::AngleAxisd(lidar_camera_rpy[0], Eigen::Vector3d::UnitZ()) * 
                            Eigen::AngleAxisd(lidar_camera_rpy[1], Eigen::Vector3d::UnitY()) * 
                            Eigen::AngleAxisd(lidar_camera_rpy[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d camera_rotation_xforward;
    camera_rotation_xforward = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()) * 
                               Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY()) * 
                               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d lidar_camera_xforward;
    lidar_camera_xforward = lidar_camera_rotation * camera_rotation_xforward;
    lidar_camera_trans_(0,0) = lidar_camera_xforward(0,0);
    lidar_camera_trans_(0,1) = lidar_camera_xforward(0,1);
    lidar_camera_trans_(0,2) = lidar_camera_xforward(0,2);
    lidar_camera_trans_(1,0) = lidar_camera_xforward(1,0);
    lidar_camera_trans_(1,1) = lidar_camera_xforward(1,1);
    lidar_camera_trans_(1,2) = lidar_camera_xforward(1,2);
    lidar_camera_trans_(2,0) = lidar_camera_xforward(2,0);
    lidar_camera_trans_(2,1) = lidar_camera_xforward(2,1);
    lidar_camera_trans_(2,2) = lidar_camera_xforward(2,2);
    lidar_camera_trans_(0,3) = ext_trans_[0];
    lidar_camera_trans_ (1,3) = ext_trans_[1];
    lidar_camera_trans_(2,3) = ext_trans_[2];
    camera_lidar_trans_ = lidar_camera_trans_.inverse();

    camera_intrinsic_ = Eigen::Matrix<float, 3, 3>::Identity();
    camera_intrinsic_(0,0) = camera_param_[0];
    camera_intrinsic_(0,1) = camera_param_[1];
    camera_intrinsic_(0,2) = camera_param_[2];
    camera_intrinsic_(1,0) = camera_param_[3];
    camera_intrinsic_(1,1) = camera_param_[4];
    camera_intrinsic_(1,2) = camera_param_[5];
    camera_intrinsic_(2,0) = camera_param_[6];
    camera_intrinsic_(2,1) = camera_param_[7];
    camera_intrinsic_(2,2) = camera_param_[8];
}

void TargetLocalization::sensorMessageCallBack( const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::PointCloud2ConstPtr& lidar_points_msg )
{
    // std::cout << "sensor init***********************************************" << std::endl; 
    cloud_header_ = lidar_points_msg->header;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_pointcloud( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::PointCloud<pcl::PointXYZI>::Ptr camera_pointcloud( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::fromROSMsg(*lidar_points_msg, *lidar_pointcloud);

    pcl::transformPointCloud( *lidar_pointcloud, *camera_pointcloud, camera_lidar_trans_);
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *camera_pointcloud, output);
    output.header.stamp = lidar_points_msg->header.stamp;
    output.header.frame_id = "camera_link";
    image_pointscloud_pub_.publish( output );

    cv_bridge::CvImageConstPtr cv_ptr_display;
    cv::Mat rgb_image;
    try
    {
        cv_ptr_display = cv_bridge::toCvShare( rgb_image_msg );
        cv::cvtColor( cv_ptr_display->image, rgb_image, CV_BGRA2BGR );
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    pointcloudRegister( rgb_image, camera_pointcloud );
}

void TargetLocalization::pointcloudRegister( cv::Mat& rgb_image, pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_pointcloud )
{
    points_image_ = cv::Mat::zeros( rgb_image.rows, rgb_image.cols, CV_32FC3 );

    Eigen::Matrix<float, 3, 1> point_vector;
    Eigen::Matrix<float, 3, 1> image_uv;
    for( pcl::PointCloud<pcl::PointXYZI>::iterator index = camera_pointcloud->begin(); index != camera_pointcloud->end(); index++ )
    {
        //Filter out point clouds behind the camera
        if( index->x <= 0 )
            continue;

        //Points in the normalized plane
        point_vector(0,0) = -index->y / index->x;
        point_vector(1,0) = -index->z / index->x;
        point_vector(2,0) = 1.0;

        image_uv = camera_intrinsic_ * point_vector;

        uint16_t u = static_cast<uint16_t>( image_uv(1,0) );
        uint16_t v = static_cast<uint16_t>( image_uv(0,0) );
        
        if( u > 0 && u < points_image_.rows && v > 0 && v < points_image_.cols )
        {
            //Here there is a coordinate system transformation that transforms the camera coordinate system to the x-forward camera coordinate system.
            points_image_.ptr<float>( u )[v * 3] = index->x;
            points_image_.ptr<float>( u )[v * 3 + 1] = index->y;
            points_image_.ptr<float>( u )[v * 3 + 2] = index->z;
        }
    }
    point_image_init_ = true;
}

void TargetLocalization::detectResultCallback( const geometry_msgs::PoseStampedConstPtr& detect_result_msg )
{
    detect_box_header_ = detect_result_msg->header;
    if( !point_image_init_ )
        return;
    //yolo_msg
    uint16_t x_min, y_min, x_max, y_max;
    x_min = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.x));
    y_min = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.y));
    x_max = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.z));
    y_max = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.w));

    targetPointcloudExtract( x_min, y_min, x_max, y_max );
}

void TargetLocalization::targetPointcloudExtract( uint16_t x_min, uint16_t y_min, int16_t x_max, uint16_t y_max )
{
    //Extract the point cloud in the recognition box
    pcl::PointCloud<pcl::PointXYZI>::Ptr detect_box_points( new pcl::PointCloud<pcl::PointXYZI>() );
    for( size_t u = y_min; u < y_max; u++ )
    {
        for( size_t v = x_min; v <  x_max; v++ )
        {
            try
            {
                pcl::PointXYZI point;
                point.x = points_image_.ptr<float>(u)[v * 3];
                point.y = points_image_.ptr<float>(u)[v * 3 + 1];
                point.z = points_image_.ptr<float>(u)[v * 3 + 2];
                double distance = sqrt( point.x * point.x + point.y * point.y + point.z * point.z );
                if( distance > 1.0 && distance < 30.0 )
                {
                    detect_box_points->push_back( point );
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            } 
        }
    }

    // pointcloud filter
    if( detect_box_points->points.size() < 30 )
        return;

    //Euclidean distance clustering
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (detect_box_points);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance ( 0.35 ); //Set the search radius of the nearest neighbor search to 35cm
    ec.setMinClusterSize ( 20 ); //Set the minimum number of points needed for a cluster to 20
    ec.setMaxClusterSize ( 2500 ); //Set the maximum number of points needed for a cluster to 2500
    ec.setSearchMethod ( tree );//Setting up the search mechanism for the point cloud
    ec.setInputCloud ( detect_box_points );
    
    ec.extract ( cluster_indices );//Extract clusters from the point cloud and save the point cloud indexes in cluster_indices
    

    uint16_t max_num = 0;
    pcl::PointIndices max_pointcloud_indice;
    for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); it++ )
    {
        if( it->indices.size() > max_num )
        {
            max_num = it->indices.size();
            max_pointcloud_indice = *it;
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr max_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
    double x_sum = 0;
    double y_sum = 0;
    double z_sum = 0;
    for( std::vector<int>::const_iterator pit = max_pointcloud_indice.indices.begin (); pit != max_pointcloud_indice.indices.end (); ++pit )
    {
        max_pointcloud->push_back( detect_box_points->points[*pit] );
        x_sum += detect_box_points->points[*pit].x;
        y_sum += detect_box_points->points[*pit].y;
        z_sum += detect_box_points->points[*pit].z;
    }
    double x_average = x_sum / max_pointcloud->points.size();
    double y_average = y_sum / max_pointcloud->points.size();
    double z_average = z_sum / max_pointcloud->points.size();
    Eigen::Vector3d point( x_average, y_average, z_average );

    //target tf filter
    resultBuf_.push_back( point );
    static double point_x_sum, point_y_sum, point_z_sum;
    static double point_x, point_y, point_z;
    if( resultBuf_.size() >= 2 )
    {
        for( size_t i = 0; i < resultBuf_.size(); i++ )
        {
            point_x_sum += resultBuf_[i][0];
            point_y_sum += resultBuf_[i][1];
            point_z_sum += resultBuf_[i][2];
        }
        point_x = point_x_sum / resultBuf_.size();
        point_y = point_y_sum / resultBuf_.size();
        point_z = point_z_sum / resultBuf_.size();
        point_x_sum = 0;
        point_y_sum = 0;
        point_z_sum = 0;
        resultBuf_.pop_front();
    }

    targetTfPublish( point_x, point_y, point_z );
    
    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg( *max_pointcloud, output2);
    output2.header.stamp = detect_box_header_.stamp;
    output2.header.frame_id = "camera_link";
    image_points_pub_.publish( output2 );
}

void TargetLocalization::targetTfPublish( double x, double y, double z )
{
    try
    {
        if( x > 0.8 && x < 30 )
        {
            tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3( x, y, z ) );
            tf::Quaternion q1;
            q1.setRPY( 0, 0, 0 );
            transform.setRotation( q1 );
            br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "camera_link", "target" ) );

            nav_goal_update_ = true;
        }
    }
    catch(std::exception e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void TargetLocalization::naveGoalPub( TargetLocalization *target_localization )
{
    ros::Publisher target_motor_position_pub_;
    target_motor_position_pub_ = target_localization->private_nh_.advertise<geometry_msgs::Point> ("target_motor_position", 1);

    std::cout << "thread starting......" << std::endl;
    ros::Rate loop_rate( 20 );
    tf::TransformListener listener_1;
    
    tf::StampedTransform transform_1, transform_2, transform_4;
    
    tf::TransformBroadcaster br;
    while( target_localization->num_ != 0 )
    {
        try
        {
            tf::TransformListener listener_2;
            tf::StampedTransform transform_motor;
            listener_2.waitForTransform( "motor_fixed", "target", ros::Time( 0 ), ros::Duration(10.0) );
            listener_2.lookupTransform( "motor_fixed", "target", ros::Time( 0 ), transform_motor );
            float X_target = transform_motor.getOrigin().getX();
            float Y_target = transform_motor.getOrigin().getY();
            float Z_target = transform_motor.getOrigin().getZ();
            geometry_msgs::Point target_position;
            target_position.x = X_target;
            target_position.y = Y_target;
            target_position.z = Z_target;

            target_motor_position_pub_.publish( target_position );
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        if( target_localization->nav_goal_update_ )
        {
            try
            {
                listener_1.waitForTransform( "base_link", "target", ros::Time( 0 ), ros::Duration(10.0) );
                listener_1.lookupTransform( "base_link", "target", ros::Time( 0 ), transform_2 );

                double X_dist = transform_2.getOrigin().getX();
                double Y_dist = transform_2.getOrigin().getY();
                double d_dist = sqrt( X_dist * X_dist + Y_dist * Y_dist );

                std::cout << "X_distance: " << X_dist << ";   ";
                std::cout << "Y_distance: " << Y_dist << ";   ";
                std::cout << "distance: " << d_dist << std::endl;
                
                if( d_dist > target_localization->distance_threshold_ )
                {
                    double x_goal = X_dist * ( 1 - 1.5 / d_dist );
                    double y_goal = Y_dist * ( 1 - 1.5 / d_dist );
                    
                    tf::Transform transform_2;
                    tf::Quaternion q2;
                    transform_2.setOrigin( tf::Vector3( x_goal, y_goal, 0.0 ) );
                    q2.setRPY( 0, 0, 0 );
                    transform_2.setRotation( q2 );
                    br.sendTransform( tf::StampedTransform( transform_2, ros::Time::now(), "base_link", "nav_goal" ) );

                    double distance = sqrt( x_goal * x_goal + y_goal * y_goal );

                    if( distance > 0.3 )
                    {
                        geometry_msgs::PoseStamped nav_goal;
                        nav_goal.header.stamp = ros::Time::now();
                        nav_goal.header.frame_id = "base_link";
                        nav_goal.pose.position.x = x_goal;
                        nav_goal.pose.position.y = y_goal;
                        nav_goal.pose.position.z = 0;

                        double roll, pitch, yaw;
                        tf::Matrix3x3(transform_1.getRotation()).getEulerYPR( yaw, pitch, roll );
                        nav_goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( yaw, pitch, roll );

                        target_localization->goal_pub_.publish( nav_goal );
                    }
                }
                else
                    std::cout << "Navigation goal is too near. d_distance :   " << d_dist << std::endl;
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR( "%s", ex.what() );
                loop_rate.sleep();
            }
            target_localization->nav_goal_update_ = false;
        }
        loop_rate.sleep();
    }  
}