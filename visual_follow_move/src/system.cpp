#include "system.h"

System::System( ros::NodeHandle &nh, ros::NodeHandle &private_nh ):nh_( nh ), private_nh_( private_nh )
{
    target_localization_ = new TargetLocalization( nh_, private_nh_ );
}

System::~System()
{
    // systemClose();
    delete target_localization_;
    // //关闭线程
    // if( serverThread->joinable() )
    // {
    //     delete serverThread;
    //     serverThread->join();
    //     // delete serverThread_2_;
    //     // serverThread_2_->join();
    // } 
}

void System::systemInit()
{
    //serverThread = new std::thread( &ServerTCP::serverRun, server );
    //serverThread_2_ = new std::thread( &ServerTCP::serverRun2, server );
}

void System::systemUpdate()
{
    // server->checkClientVector(); //监视客户端列表
    ros::spinOnce(); //检测回调函数
}

void System::systemClose()
{
    //server->setRunStatus( false );
}