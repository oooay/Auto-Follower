#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"


namespace yhs_tool {


CanControl::CanControl():
x_cur_(0.0), y_cur_(0.0), z_cur_(0.0), theta_cur_(0.0), cur_pulse_left_(0), cur_pulse_right_(0), 
pre_pulse_left_(0), pre_pulse_right_(0), private_node_("~"), velocity_(0), alpha_(0)
{
	ros::NodeHandle private_node("~");
	
	//oay
	private_node_.param<std::string>( "twiststamped_pubtopic_name", twiststamped_pubtopic_name_, "/current_velocity" );
	private_node_.param<std::string>( "vehicle_odom", vehicle_odom_, "/vehicle/odom" );
	private_node_.param<std::string>( "twiststamped_subtopic_name", twiststamped_subtopic_name_, "/twist_cmd" );
	private_node_.param<std::string>( "twist_subtopic_name", twist_subtopic_name_, "/twist_cmd/twist" );
	private_node_.param<double>( "wheel_base", wheel_base_, 0.66 );
	private_node_.param<double>( "wheel_distance", wheel_distance_, 0.606 );
}


CanControl::~CanControl()
{

}

//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = msg.io_cmd_enable;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0;
	short angular = msg.ctrl_cmd_steering * 100;
	static unsigned char count = 0;

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);

	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel_[1] = (vel >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	sendData_u_vel_[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  count << 4;
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//数据接收解析线程
void CanControl::recvData()
{

	while(ros::ok())
	{

		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
		{
			for(int j=0;j<1;j++)
			{
				
				switch (recv_frames_[0].can_id)
				{
					//速度控制反馈
					case 0x98C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						
						msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;
						
						msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							ctrl_fb_pub_.publish(msg);
						}

						break;
					}

					//左轮反馈
					case 0x98C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
							//oay
							left_wheel_ = msg.lr_wheel_fb_velocity;
							cur_pulse_left_ = msg.lr_wheel_fb_pulse;
							static bool firstData = true;
							if( firstData )
							{
								pre_pulse_left_ = cur_pulse_left_;
								firstData = false;
							}
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//右轮反馈
					case 0x98C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
							//oay
							right_wheel_ = msg.rr_wheel_fb_velocity;
							cur_pulse_right_ = msg.rr_wheel_fb_pulse;
							static bool firstData = true;
							if( firstData )
							{
								pre_pulse_right_ = cur_pulse_right_;
								firstData = false;
							}
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x98C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//里程计反馈
					case 0x98C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					//bms_Infor反馈
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_Infor_fb msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag_Infor反馈
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor_fb msg;
						msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//Drive_fb_MCUEcoder反馈
					case 0x98C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						msg.Drive_fb_MCUEcoder = (int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]); 

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					//Veh_fb_Diag反馈
					case 0x98C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

						if(0x10 & recv_frames_[0].data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & recv_frames_[0].data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisOnline = true;	else msg.Veh_fb_EPSDisOnline = false;

						if(0x02 & recv_frames_[0].data[1]) msg.Veh_fb_EPSfault = true;	else msg.Veh_fb_EPSfault = false;

						if(0x04 & recv_frames_[0].data[1]) msg.Veh_fb_EPSMosfetOT = true;	else msg.Veh_fb_EPSMosfetOT = false;

						if(0x08 & recv_frames_[0].data[1]) msg.Veh_fb_EPSWarning = true;	else msg.Veh_fb_EPSWarning = false;

						if(0x10 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisWork = true;	else msg.Veh_fb_EPSDisWork = false;

						if(0x20 & recv_frames_[0].data[1]) msg.Veh_fb_EPSOverCurrent = true;	else msg.Veh_fb_EPSOverCurrent = false;

						
						
						if(0x10 & recv_frames_[0].data[2]) msg.Veh_fb_EHBecuFault = true;	else msg.Veh_fb_EHBecuFault = false;

						if(0x20 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisOnline = true;	else msg.Veh_fb_EHBDisOnline = false;

						if(0x40 & recv_frames_[0].data[2]) msg.Veh_fb_EHBWorkModelFault = true;	else msg.Veh_fb_EHBWorkModelFault = false;

						if(0x80 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisEn = true;	else msg.Veh_fb_EHBDisEn = false;


						if(0x01 & recv_frames_[0].data[3]) msg.Veh_fb_EHBAnguleFault = true;	else msg.Veh_fb_EHBAnguleFault = false;

						if(0x02 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOT = true;	else msg.Veh_fb_EHBOT = false;

						if(0x04 & recv_frames_[0].data[3]) msg.Veh_fb_EHBPowerFault = true;	else msg.Veh_fb_EHBPowerFault = false;

						if(0x08 & recv_frames_[0].data[3]) msg.Veh_fb_EHBsensorAbnomal = true;	else msg.Veh_fb_EHBsensorAbnomal = false;

						if(0x10 & recv_frames_[0].data[3]) msg.Veh_fb_EHBMotorFault = true;	else msg.Veh_fb_EHBMotorFault = false;

						if(0x20 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilPressSensorFault = true;	else msg.Veh_fb_EHBOilPressSensorFault = false;

						if(0x40 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilFault = true;	else msg.Veh_fb_EHBOilFault = false;




						if(0x01 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUDisOnline = true;	else msg.Veh_fb_DrvMCUDisOnline = false;

						if(0x02 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOT = true;	else msg.Veh_fb_DrvMCUOT = false;

						if(0x04 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOV = true;	else msg.Veh_fb_DrvMCUOV = false;

						if(0x08 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUUV = true;	else msg.Veh_fb_DrvMCUUV = false;

						if(0x10 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUShort = true;	else msg.Veh_fb_DrvMCUShort = false;

						if(0x20 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUScram = true;	else msg.Veh_fb_DrvMCUScram = false;

						if(0x40 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUHall = true;	else msg.Veh_fb_DrvMCUHall = false;

						if(0x80 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUMOSFEF = true;	else msg.Veh_fb_DrvMCUMOSFEF = false;


						if(0x10 & recv_frames_[0].data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & recv_frames_[0].data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					default:
						break;
				}

			}

			//oay
			pubCurrentVelocity();
			pubOdom();
					
		}
	}
}

//oay
void CanControl::pubCurrentVelocity()
{
	cmd_mutex_.lock();
	speed_.header.stamp = ros::Time::now();
	speed_.twist.linear.x = ( left_wheel_ + right_wheel_ ) / 2.0;
	speed_.twist.angular.z = ( right_wheel_ - left_wheel_ ) / WHEEL_BASE;
	speed_.header.frame_id = "base_link";
	cur_vel_Pub_.publish( speed_ );

	cmd_mutex_.unlock();
}

//oay
void CanControl::pubOdom()
{
	cmd_mutex_.lock();

	//计算左右轮
	int32_t d_pulse_left = cur_pulse_left_ - pre_pulse_left_;
	int32_t d_pulse_right = cur_pulse_right_ - pre_pulse_right_;

	tf::Quaternion q1;

	pre_pulse_left_ = cur_pulse_left_;
	pre_pulse_right_ = cur_pulse_right_;
	double distance_left = d_pulse_left / PULSE_NUM * PI * WHEEL_D;
	double distance_right = d_pulse_right / PULSE_NUM * PI * WHEEL_D;

	double d_distance = ( distance_left + distance_right ) / 2.0;
	double d_thetha = ( distance_right - distance_left ) / WHEEL_BASE;
	double d_x = cos( d_thetha ) * d_distance;
	double d_y = sin( d_thetha ) * d_distance;

	x_cur_ += cos( theta_cur_ ) * d_x - sin( theta_cur_ ) * d_y;
	y_cur_ += sin( theta_cur_ ) * d_x + cos( theta_cur_ ) * d_y;
	theta_cur_ += d_thetha;
	if( theta_cur_ > PI * 2 )
		theta_cur_ -= PI * 2;
	if( theta_cur_ < -PI * 2 )
		theta_cur_ += PI * 2;
	//std::cerr << theta_cur_ << std::endl;

	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.pose.pose.position.x = x_cur_;
    odom.pose.pose.position.y = y_cur_;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0, theta_cur_ );
	odom.twist.twist = speed_.twist;
	cur_odom_Pub_.publish( odom );

	transform.setOrigin( tf::Vector3( x_cur_, y_cur_, 0 ) );
	q1.setRPY( 0, 0, theta_cur_ );
	transform.setRotation( q1 );
	br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "vehicle_odom", "vehicle_link" ) );

	cmd_mutex_.unlock();
}

//oay
void CanControl::twistCmdCallBack( const geometry_msgs::TwistConstPtr &twist )
{
	yhs_can_msgs::ctrl_cmd ctrl_c;
	if( abs( twist->linear.x ) > 0 )
	{  
		velocity_ = twist->linear.x;
		if( abs( twist->angular.z ) < 0.001 )
		{
			alpha_ = 0;
		}
		else
		{
			double radius = abs( twist->linear.x / twist->angular.z );
			alpha_ = atan2( wheel_base_, ( radius - wheel_distance_ / 2.0 ) );
		}

		if( twist->angular.z > 0 && twist->linear.x > 0 )
			ctrl_c.ctrl_cmd_steering = alpha_ * 180 / 3.14;
		else if( twist->angular.z < 0 && twist->linear.x > 0 )
			ctrl_c.ctrl_cmd_steering = -alpha_ * 180 / 3.14;
		else if( twist->angular.z > 0 && twist->linear.x < 0 )
			ctrl_c.ctrl_cmd_steering = -alpha_ * 180 / 3.14;
		else if( twist->angular.z < 0 && twist->linear.x < 0 )
			ctrl_c.ctrl_cmd_steering = alpha_ * 180 / 3.14;
		else
			ctrl_c.ctrl_cmd_steering = 0;

		if( twist->linear.x > 0 )
		{
			ctrl_c.ctrl_cmd_gear = 4;
			ctrl_c.ctrl_cmd_velocity = velocity_;
		}
		else if( twist->linear.x < 0 )
		{
			ctrl_c.ctrl_cmd_gear = 2;
 			ctrl_c.ctrl_cmd_velocity = -velocity_;
		}
		else
		{
			ctrl_c.ctrl_cmd_gear = 1;
    		ctrl_c.ctrl_cmd_velocity = 0;
		}
	}
	else
	{
		ctrl_c.ctrl_cmd_velocity = 0;
	}

	ctrl_cmdCallBack( ctrl_c );
}

//oay
void CanControl::twistStampedCallBack( const geometry_msgs::TwistStampedConstPtr &twist_stamped )
{
	yhs_can_msgs::ctrl_cmd ctrl_c;
	if( abs( twist_stamped->twist.linear.x ) > 0 )
	{  
		//std::cout << ++num << ":  " << twist_stamped->twist.linear.x << std::endl;
		velocity_ = twist_stamped->twist.linear.x;
		if( abs( twist_stamped->twist.angular.z ) < 0.001 )
		{
			alpha_ = 0;
		}
		else
		{
			double radius = abs( twist_stamped->twist.linear.x / twist_stamped->twist.angular.z );
			alpha_ = atan2( wheel_base_, ( radius - wheel_distance_ / 2.0 ) );
		}

		if( twist_stamped->twist.angular.z > 0 && twist_stamped->twist.linear.x > 0 )
			ctrl_c.ctrl_cmd_steering = alpha_ * 180 / 3.14;
		else if( twist_stamped->twist.angular.z < 0 && twist_stamped->twist.linear.x > 0 )
			ctrl_c.ctrl_cmd_steering = -alpha_ * 180 / 3.14;
		else if( twist_stamped->twist.angular.z > 0 && twist_stamped->twist.linear.x < 0 )
			ctrl_c.ctrl_cmd_steering = -alpha_ * 180 / 3.14;
		else if( twist_stamped->twist.angular.z < 0 && twist_stamped->twist.linear.x < 0 )
			ctrl_c.ctrl_cmd_steering = alpha_ * 180 / 3.14;
		else
			ctrl_c.ctrl_cmd_steering = 0;

		if( twist_stamped->twist.linear.x > 0 )
		{
			ctrl_c.ctrl_cmd_gear = 4;
			ctrl_c.ctrl_cmd_velocity = velocity_;
		}
		else if( twist_stamped->twist.linear.x < 0 )
		{
			ctrl_c.ctrl_cmd_gear = 2;
 			ctrl_c.ctrl_cmd_velocity = -velocity_;
		}
		else
		{
			ctrl_c.ctrl_cmd_gear = 1;
    		ctrl_c.ctrl_cmd_velocity = 0;
		}
	}
	else
	{
		ctrl_c.ctrl_cmd_velocity = 0;
	}

	//std::cout << ++num << ":  " << ctrl_c.ctrl_cmd_velocity << std::endl;
	if(num > 65535) num = 0;

	ctrl_cmdCallBack( ctrl_c );
}


//数据发送线程
void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}

}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb",5);
	bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb",5);
	bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5);

	//oay
	cur_vel_Pub_ = nh_.advertise<geometry_msgs::TwistStamped>( twiststamped_pubtopic_name_, 5 );
	cur_odom_Pub_ = nh_.advertise<nav_msgs::Odometry>(vehicle_odom_, 5);
	twist_stamped_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>( twiststamped_subtopic_name_, 5, &CanControl::twistStampedCallBack, this );
	twist_sub_ = nh_.subscribe<geometry_msgs::Twist>( twist_subtopic_name_, 5, &CanControl::twistCmdCallBack, this );

	//打开设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}


	struct ifreq ifr;
	
	std::string can_name("can0");

	strcpy(ifr.ifr_name,can_name.c_str());

	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);


    // bind socket to network interface
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	ros::spin();
	
	close(dev_handler_);
}

}



//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
