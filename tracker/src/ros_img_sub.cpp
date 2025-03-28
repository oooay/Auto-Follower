/*
ros订阅图像话题，并进行逐帧处理
*/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
// #include "image_transport/image_transport.h"


void imgCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr ptr;
    ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat img = ptr->image;
    // cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::circle(img, cv::Point(20, 100), 20, cv::Scalar(2, 0, 255), -1);
    cv::imshow("img", img);
    cv::waitKey(30); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle n;
    ros::Subscriber img_sub = n.subscribe("zed_node/left/image_rect_color", 1, imgCallback);
    ros::spin();
    cv::destroyAllWindows();   

    return 0;
}
 
