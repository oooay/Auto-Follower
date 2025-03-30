#include "system.h"

System::System( ros::NodeHandle &nh, ros::NodeHandle &private_nh ):nh_( nh ), private_nh_( private_nh )
{
    target_localization_ = new TargetLocalization( nh_, private_nh_ );
}

System::~System()
{
    delete target_localization_;
}

void System::systemInit()
{
}

void System::systemUpdate()
{
    ros::spinOnce(); //检测回调函数
}

void System::systemClose()
{
}