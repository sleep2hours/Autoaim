#ifndef _READ_PARAM_HPP
#define _READ_PARAM_HPP

#include <ros/ros.h>
#include <iostream>

template <typename T>
T readParam(ros::NodeHandle &n, std::string name){              //参数服务器读取
    T ans;
    if (n.getParam(name, ans)) {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else{
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans; 
}


#endif  //_READ_PARAM_HPP
