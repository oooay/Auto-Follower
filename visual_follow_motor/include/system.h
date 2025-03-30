#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <iostream>
#include "server_tcp.h"
#include "client_socket.h"
#include <thread>
#include "future_twist_to_serial.h"
#include "ros/ros.h"

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
    ServerTCP *server;
    std::thread *serverThread; //server thread
    FutureSerial *futureSerial; //serial port object (computing)
};

#endif