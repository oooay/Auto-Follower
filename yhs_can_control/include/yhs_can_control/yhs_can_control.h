#ifndef __PIDCONTROL_NODE_H__
#define __PIDCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor_fb.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TwistStamped.h>

#include <iostream>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//oay
#define WHEEL_BASE 0.606//车轮轮距
#define WHEEL_D 0.42 //车轮直径
#define PULSE_NUM 400.0 //车轮编码器线数
#define PI 3.14159

namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();

	//oay
	void pubCurrentVelocity();
	void pubOdom();
	void twistCmdCallBack( const geometry_msgs::TwistConstPtr &twist );
	void twistStampedCallBack( const geometry_msgs::TwistStampedConstPtr &twist_stamped );
private:
	ros::NodeHandle nh_;

	ros::Publisher ctrl_fb_pub_;
	ros::Publisher lr_wheel_fb_pub_;
	ros::Publisher rr_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher odo_fb_pub_;
	ros::Publisher bms_Infor_fb_pub_;
	ros::Publisher bms_flag_Infor_fb_pub_;
	ros::Publisher Drive_MCUEcoder_fb_pub_;
	ros::Publisher Veh_Diag_fb_pub_;

	//oay
	ros::Publisher cur_vel_Pub_;
	ros::Publisher cur_odom_Pub_;
	ros::Subscriber twist_stamped_sub_;
	ros::Subscriber twist_sub_;
	double left_wheel_;
	double right_wheel_;
	double velocity_, alpha_;
	double wheel_base_, wheel_distance_;
	geometry_msgs::TwistStamped speed_;
	std::string twiststamped_pubtopic_name_;
	std::string vehicle_odom_;
	std::string twist_subtopic_name_;
	std::string twiststamped_subtopic_name_;
	//oay
	double x_cur_, y_cur_, z_cur_, theta_cur_;
	int32_t cur_pulse_left_, cur_pulse_right_;
	int32_t pre_pulse_left_, pre_pulse_right_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;

	//oay
	ros::NodeHandle private_node_;

	boost::mutex cmd_mutex_;

	unsigned char sendData_u_io_[8] = {0};
	unsigned char sendData_u_vel_[8] = {0};

	int dev_handler_;
	can_frame send_frames_[2];
	can_frame recv_frames_[1];


	void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);


	void recvData();
	void sendData();

	//oay
	uint32_t num = 0;
	tf::TransformBroadcaster br;
	tf::Transform transform;

};

}


#endif

