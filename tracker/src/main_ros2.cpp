#include "chrono"
#include "yolov8.hpp"
#include "opencv2/opencv.hpp"
#include "BYTETracker.h"
#include "STrack.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "track/Box.h"
#include "track/BoxArray.h"

int resetTime = 2;
int FPS = 30;
int lost_time = 100;
double track_th = 0.1;
double high_th;
float match_th;
// const std::string weight = "/home/forerun/catkin/catkin_fdu/src/track2/weights/fuyunv8n.engine";

const std::string weight = "/home/titan/catkin_fdu/src/track/weights/fuyunv8n-fp16.engine";
cv::Size yoloSize = cv::Size{640, 640};
std::vector<Object> objs;
ros::Publisher targetbox_pub;
ros::Publisher boxes_pub;
int target_id = -1;
vector<int> prev_tlbr(4, 0.0);
int reset_count = 0;
bool isReset{false};

auto yolov8 = new YOLOv8(weight);
auto tracker = new BYTETracker(track_th, high_th, match_th, FPS, lost_time);

void idCallback(const std_msgs::Int32::ConstPtr &msg)
{
    target_id = msg->data;
}

void imgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat img = ptr->image;
	ROS_INFO("=======run step=======\n");
    auto trackerstart1 = std::chrono::system_clock::now();
    objs.clear();
    yolov8->copy_from_Mat(img, yoloSize);
    // infer
    auto inferstart = std::chrono::system_clock::now();
    yolov8->infer();
    auto inferend = std::chrono::system_clock::now();
    // ROS_INFO("infer time cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(inferend - inferstart).count() / 1000.);
    yolov8->postprocess(objs);			
    // ROS_INFO("reset tracker flag is : %d\n", isReset);
    // track
    auto trackstart = std::chrono::system_clock::now();
    int imgWidth = img.size[1];
    int imgHeight = img.size[0];
    int widthStep = imgWidth / 3;
    vector<STrack> output_stracks = tracker->update(objs, isReset, target_id, widthStep);
    isReset = false;
    auto trackend = std::chrono::system_clock::now();
    ROS_INFO("tracker cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(trackend - trackstart).count() / 1000.);
    geometry_msgs::PoseStamped target_box;
    track::BoxArray boxes;
    for (int i = 0; i < output_stracks.size(); i++)
    {
        vector<float> tlwh = output_stracks[i].tlwh;
        int x1 = int(tlwh[0]);
        int y1 = int(tlwh[1]);
        int x2 = int(tlwh[0] + tlwh[2]);
        int y2 = int(tlwh[1] + tlwh[3]);
        vector<int> tlbr = {x1, y1, x2, y2};
        bool vertical = tlwh[2] / tlwh[3] > 1.6; // 横纵比
        int id = output_stracks[i].track_id;
        Scalar color = tracker->get_color(id);
        string puttext_id = std::to_string(id);
        cv::putText(img, puttext_id, cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, color, 2, cv::LINE_AA);
        cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), color, 2);
        if (id != target_id && x1 > widthStep && x2 < widthStep * 2){
            float iou = tracker->iou(prev_tlbr, tlbr);
            if (iou > 0.9){
                reset_count++;
            }
            else {
                prev_tlbr = tlbr;
            }
        }
        // ROS_INFO("reset_frame is : %d\n", reset_count);
        if (reset_count > resetTime * 30) {
            isReset = true;
            reset_count = 0;
        }

        if(target_id > 0 && id == target_id)
        {
            target_box.header.stamp = ros::Time::now();
            target_box.pose.orientation.x = x1;
            target_box.pose.orientation.y = y1;
            target_box.pose.orientation.z = x2;
            target_box.pose.orientation.w = y2;
            targetbox_pub.publish(target_box);
        }
        track::Box box;
        box.box_id = id;
        box.top_left_x = x1;
        box.top_left_y = y1;
        box.bottom_right_x = x2;
        box.bottom_right_y = y2;
        boxes.boxes.push_back(box);	
    }
    boxes_pub.publish(boxes);
    cv::imshow("Tracker", img);
    cv::waitKey(30); 
}

int main(int argc, char** argv)
{
    cudaSetDevice(0);
    yolov8->make_pipe(true);
    cv::namedWindow("Tracker", cv::WINDOW_NORMAL);
    cv::resizeWindow("Tracker", 1280, 720);
	ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;
    FPS = n.param("camera_node/FPS", 30);
    resetTime = n.param("camera_node/resetTime", 5);
    lost_time = n.param("camera_node/lostTime", 100);
    track_th = n.param("camera_node/track_th", 0.2);
    high_th = n.param("camera_node/high_th", 0.8);
    match_th = n.param("camera_node/match_th", 1.0);
    
    ros::Subscriber img_sub = n.subscribe("zed_node/left/image_rect_color", 10, imgCallback);
    ros::Subscriber targetid_sub = n.subscribe("vision_detector/id_result", 10, idCallback);
    targetbox_pub = n.advertise<geometry_msgs::PoseStamped>("vision_detector/box_result", 10);
    boxes_pub = n.advertise<track::BoxArray>("vision_detector/boxes_result", 10);
    ros::spin();
	cv::destroyAllWindows();
	delete yolov8;
	delete tracker;

	return 0;
}
