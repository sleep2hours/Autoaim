#ifndef AIM_ROS_HPP
#define AIM_ROS_HPP

#include <ros/ros.h>
#include <iostream>
#include "serial_com/comm.h"

/// 主函数不使用全局变量而使用类成`员callBack，数据更加稳定

class AimRos{
public:
    AimRos(){
        g_msg.x      = 0;
        g_msg.y      = 0;
        g_msg.z      = 0;
        g_msg.status = 0;
        oldx = 0;
        oldy = 0;        
        oldz = 0;        
        old_stat = 0;
        spin_flag = false;
    }
    ~AimRos(){;}
public:
    void subCallback(const serial_com::comm::ConstPtr &msg){
        g_msg.x             = msg->x;
        g_msg.y             = msg->y;
        g_msg.z             = msg->z;
        g_msg.status        = msg->status;
        oldx                = msg->x;
        oldy                = msg->y;
        oldz                = msg->z;
        old_stat            = msg->status;
        spin_flag = true;
    }

    void updateUseOld() {
        if (spin_flag == false){
            g_msg.x = oldx;
            g_msg.y = oldy;
            g_msg.z = oldz;
            g_msg.status = old_stat;
        }
    }

    void clearSpinFlag() {
        this->spin_flag = false;
    }

public:
    bool spin_flag;
    float oldx;
    float oldy;
    float oldz;
    float old_stat;
    serial_com::comm g_msg;
};

#endif  //AIM_ROS_HPP