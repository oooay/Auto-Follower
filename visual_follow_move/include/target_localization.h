#ifndef TARGET_LOCALIZATION_H
#define TARGET_LOCALIZATION_H

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <mutex>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

class TargetLocalization
{
    public:
    TargetLocalization( ros::NodeHandle &nh, ros::NodeHandle &private_nh );
    ~TargetLocalization();
    static void naveGoalPub( TargetLocalization *target_localization );
    void paramInit();
    void sensorMessageCallBack( const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::PointCloud2ConstPtr& lidar_points_cloud );
    void detectResultCallback( const geometry_msgs::PoseStampedConstPtr& detect_result_msg );
    void targetTfPublish( double x, double y, double z );
    void intrinsicExtrinsicInit();
    void pointcloudRegister( cv::Mat& rgb_image, pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_pointcloud );
    void targetPointcloudExtract( uint16_t x_min, uint16_t y_min, int16_t x_max, uint16_t y_max );

    private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher goal_pub_;
    ros::Publisher target_localization_pub;
    ros::Publisher image_pointscloud_pub_;
    ros::Publisher image_points_pub_;
    ros::Subscriber detect_result_sub_;

    ros::Publisher target_motor_position_pub_;
    
    
    //Creating a topic subscription object
    image_transport::ImageTransport n;
    image_transport::Publisher points_image_pub_;

    uint16_t x_min_, y_min_, x_max_, y_max_;

    double distance_threshold_; // oay default:3.0

    std::thread* nav_goal_thread_; //Creating a frame thread
    std::mutex m; //Instantiate m objects, thread mutexes

    int num_ = 1;
    std::deque<Eigen::Vector3d> resultBuf_; 

    message_filters::Subscriber<sensor_msgs::Image> *rgb_image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_points_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> sync_pol_1_;
    message_filters::Synchronizer<sync_pol_1_> *sync_1_;

    Eigen::Matrix4f lidar_camera_trans_; //lidar to camera transform
    Eigen::Matrix4f camera_lidar_trans_; //camera to lidar transform
    Eigen::Matrix<float, 3, 3> camera_intrinsic_; //camera intrinsic
    cv::Mat points_image_;
    bool nav_goal_update_;
    bool point_image_init_;
    std_msgs::Header cloud_header_, detect_box_header_;

    //Topic
    std::string pointcloud_topic_;
    std::string image_topic_;
    std::string detect_box_topic_;

    //lidar->camera
    std::vector<double> ext_rot_;
    std::vector<double> ext_trans_;
    std::vector<double> camera_param_;
};

#endif