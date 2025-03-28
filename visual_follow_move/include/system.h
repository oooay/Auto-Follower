#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <iostream>
#include "target_localization.h"
#include <thread>

class System
{
    public:
    System( ros::NodeHandle &nh, ros::NodeHandle &private_nh );
    ~System();
    
    void systemInit();
    void systemUpdate();
    void systemClose();

    private:
    ros::NodeHandle nh_, private_nh_;
    TargetLocalization *target_localization_;
};

#endif