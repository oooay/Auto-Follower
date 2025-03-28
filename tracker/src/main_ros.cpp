#include "chrono"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Image.h"
#include "yolov8.hpp"
#include "opencv2/opencv.hpp"
#include "BYTETracker.h"
#include "STrack.h"
#include "Worker.h"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"

Worker::Worker()
{
    img_sub = n.subscribe("zed_node/left/image_rect_color", 10, &Worker::imgCallback, this);
    targetid_sub = n.subscribe("vision_detector/id_result", 10, &Worker::idCallback, this);
    targetbox_pub = n.advertise<geometry_msgs::PoseStamped>("vision_detector/box_result", 10);
    boxes_pub = n.advertise<tracker::BoxArray>("vision_detector/boxes_result", 10);
	const std::string weight = n.param<std::string>("camera_node/weight_path", "src/track/weights/fuyunv8n-fp16.engine");
	yolo = new YOLOv8(weight);
	yolo->make_pipe(true);
	target_id = 1;
	reset_count = 0;
	prev_tlbr.resize(4);
    resetIOU = n.param("camera_node/resetIOU", 0.9);
	resetTime = n.param("camera_node/resetTime", 2);
    resetCenter = n.param("camera_node/resetCenter", 100);
	y_th = n.param("camera_node/y_th", 0.8);	
    float track_th = n.param("camera_node/track_th", 0.2);
    float high_th = n.param("camera_node/high_th", 0.8);
    float match_th = n.param("camera_node/match_th", 1.0);
    FPS = n.param("camera_node/FPS", 30);
    int lostTime = n.param("camera_node/lostTime", 100);
	tracker = new BYTETracker(track_th, high_th, match_th, FPS, lostTime);
}

Worker::~Worker()
{

}

void Worker::idCallback(const std_msgs::Int32::ConstPtr &msg)
{
    target_id = msg->data;
}

void Worker::imgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat img = ptr->image;
	cv::Size yoloSize = cv::Size{640, 640};
    yolo->copy_from_Mat(img, yoloSize);
    // infer
    auto inferstart = std::chrono::system_clock::now();
    yolo->infer();
    auto inferend = std::chrono::system_clock::now();
    // ROS_INFO("infer time cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(inferend - inferstart).count() / 1000.);
    std::vector<Object> objs;
	yolo->postprocess(objs);			
    // ROS_INFO("reset tracker flag is : %d\n", isReset);
    // track
    auto trackstart = std::chrono::system_clock::now();
    int imgWidth = img.size[1];
    int imgHeight = img.size[0];
    vector<STrack> output_stracks = tracker->update(objs, isReset, target_id, imgWidth, imgHeight, resetCenter, y_th);
    isReset = false;
    auto trackend = std::chrono::system_clock::now();
    // ROS_INFO("tracker cost %2.4lf ms\n", chrono::duration_cast<chrono::microseconds>(trackend - trackstart).count() / 1000.);
    geometry_msgs::PoseStamped target_box;
    tracker::BoxArray boxes;
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
        if (id != target_id && x1 > imgWidth / 2 - resetCenter && x2 < imgWidth / 2 + resetCenter){
            float iou = tracker->iou(prev_tlbr, tlbr);
            if (iou > resetIOU){
                if (output_stracks.size() == 1) reset_count += 5;
                else reset_count++;
            }
            else {
                prev_tlbr = tlbr;
                reset_count = max(0, reset_count - 1);       
            }
        }
        // ROS_ERROR("reset_frame is : %d\n", reset_count);
		// ROS_ERROR("target id is : %d\n", target_id);
        if (reset_count >= resetTime * FPS) {
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
        tracker::Box box;
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
	cv::namedWindow("Tracker", cv::WINDOW_NORMAL);
    cv::resizeWindow("Tracker", 1280, 720);
	ros::init(argc, argv, "camera_node");
	Worker worker;
    ros::spin();
    cv::destroyAllWindows();

	return 0;
}
