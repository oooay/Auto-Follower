#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "yolov8.hpp"
#include "BYTETracker.h"
#include "geometry_msgs/PoseStamped.h"
#include "tracker/Box.h"
#include "tracker/BoxArray.h"

class Worker
{
private:
    ros::NodeHandle n;
    ros::Subscriber img_sub, targetid_sub;
    bool isReset{false};
    int target_id{-1};
    int FPS;
    float resetIOU;
    int reset_count, resetTime, resetCenter;
    float y_th;
    vector<int> prev_tlbr;

    // auto track;
public:
    YOLOv8* yolo;
    BYTETracker* tracker;
    Worker();
    ~Worker();
    void imgCallback(const sensor_msgs::ImageConstPtr &msg);
    void idCallback(const std_msgs::Int32::ConstPtr &msg);
    ros::Publisher targetbox_pub, boxes_pub;
};
