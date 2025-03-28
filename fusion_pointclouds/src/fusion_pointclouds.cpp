#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class fusion_pointclouds
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Publisher pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr_;

    void initializeSubscribers();
    void initializePublishers();
    void callback1(const sensor_msgs::PointCloud2ConstPtr &cloud);
    void callback2(const sensor_msgs::PointCloud2ConstPtr &cloud);

public:
    fusion_pointclouds(ros::NodeHandle *node);
    ~fusion_pointclouds();
};

fusion_pointclouds::fusion_pointclouds(ros::NodeHandle *node) : nh_(*node),
                                                                transformed_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZI>())
{
    initializeSubscribers();
    initializePublishers();
}

fusion_pointclouds::~fusion_pointclouds()
{
}

void fusion_pointclouds::initializeSubscribers()
{
    sub1_ = nh_.subscribe("livox/lidar", 1, &fusion_pointclouds::callback1, this);
    sub2_ = nh_.subscribe("rslidar_points", 1, &fusion_pointclouds::callback2, this);
}

void fusion_pointclouds::initializePublishers()
{
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("fusion_points", 1);
}

void fusion_pointclouds::callback1(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud, *temp_cloud_ptr);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.035, 0, 0.16;
    transform.rotate(Eigen::AngleAxisf(3.1415926, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.05, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*temp_cloud_ptr, *transformed_cloud_ptr_, transform);
}

void fusion_pointclouds::callback2(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr fusion_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud, *fusion_cloud_ptr);

    for (int i = 0; i < transformed_cloud_ptr_->points.size(); i++)
    {
        if (transformed_cloud_ptr_->points[i].x*transformed_cloud_ptr_->points[i].x+transformed_cloud_ptr_->points[i].y*transformed_cloud_ptr_->points[i].y>9 && transformed_cloud_ptr_->points[i].z < -0.3)
        {
            continue;
        }
        if (transformed_cloud_ptr_->points[i].x*transformed_cloud_ptr_->points[i].x+transformed_cloud_ptr_->points[i].y*transformed_cloud_ptr_->points[i].y>25 && transformed_cloud_ptr_->points[i].z < -0.15)
        {
            continue;
        }
        fusion_cloud_ptr->points.push_back(transformed_cloud_ptr_->points[i]);
    }

    fusion_cloud_ptr->width = fusion_cloud_ptr->points.size();
    fusion_cloud_ptr->height = 1;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*fusion_cloud_ptr, msg);
    msg.header.frame_id = "velodyne";

    pub_.publish(msg);
    std::cout << "fusion points success." << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_pointclouds");
    ros::NodeHandle nh;

    fusion_pointclouds fusion_pointclouds(&nh);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
