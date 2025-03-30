#ifndef __VISUAL_FOLLOW_MOTOR
#define __VISUAL_FOLLOW_MOTOR

#include <serial/serial.h>
#include <ros/ros.h>
#include <string>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>

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

#include "std_msgs/Int32.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <unistd.h>

#include <vector>

#define ANGULAR_THNRESHOLD 175
#define CENTER_X 640

class MotorSerial
{
    public:
    MotorSerial( ros::NodeHandle &nh, ros::NodeHandle &private_nh );
    ~MotorSerial();
    uint8_t motorSerialStart(); //start the serial port (computing)
    void serialDataReading();
    inline void serialDataProcessing( uint8_t *buf, size_t data_size );
    void getMotorRotation();
    void motorPositionControlCmd( uint32_t angle, uint8_t spin_direction, uint32_t speed );
    void motorSpeedControlCmd( int32_t speed_control );
    void currCloudHandler( const sensor_msgs::PointCloud2ConstPtr& msg );
    void positionControlModel();
    void speedControlModel();
    void angleControlModel();

    void detectResultCallback( const geometry_msgs::PoseStampedConstPtr& detect_result_msg );
    inline void angleCompute( const uint8_t *buf, int32_t i );
    void statusCheck(); //System status monitoring

    private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber detect_result_sub_;
    ros::Subscriber sub_pointcloud_; // Declare a Subscriber to receive the current frame of the point cloud topic.
    ros::Publisher fixed_points_pub_; //Motor Fixed End Point Cloud Release
    ros::Publisher motor_angle_pub_;

    serial::Serial motorSerial_;
    std::string serialPortNum_; //serial number
    int32_t serialBaudrate_; //baudrate
    std::string detect_box_topic_;
    int32_t current_angular_; //-90 - 90

    tf::TransformBroadcaster br_;

    Eigen::Matrix4f fixed_free_trans_; //motor_fixed to motor_free transform

    uint16_t time_num_;

    bool search_status_; //Search Mode Switch

    double fx_, cx_;

    int16_t center_x_;

    bool target_update_;
};

#endif