#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include "include/serial_com/Simulate.hpp"

template <typename T>
T readParam(ros::NodeHandle &n, std::string name){              //参数服务器读取
    T ans;
    if (n.getParam(name, ans)) {
        std::cout <<"Loaded" << name << ":" << ans << std::endl;
    }
    else{
       std::cout <<"Failed to load:" << name << std::endl;
        n.shutdown();
    }
    return ans;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh;
    int mode = readParam<int>(nh, "/debug_mode");
    int is_blue = readParam<int>(nh, "/init_color_blue");
    sim::Simulate sml(nh, mode, is_blue);
    ros::spin();
    return 0;
}
