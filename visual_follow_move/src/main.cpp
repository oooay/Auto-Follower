#include <iostream>
#include <ros/ros.h>
#include "system.h"

int main( int argc, char** argv )
{
    ros::init( argc, argv, "visual_follow_move" );
    ros::NodeHandle nh, private_nh( "~" );

    System system( nh, private_nh );
    system.systemInit();

    ros::Rate loop_rate( 50 );
    while( ros::ok() )
    {
        system.systemUpdate();
        loop_rate.sleep();
    }

    system.systemClose(); //结束系统

    return 0;
}