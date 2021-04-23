/*
串口通信模块
作者：hqy
*/

#ifndef _SERIAL_COM_HPP
#define _SERIAL_COM_HPP
/** TODO:改了通信协议，所以有很多地方要改*/
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <std_msgs/String.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <opencv2/highgui.hpp>
#include "serial_com/comm.h"
// #define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
    #define serial_debug printf
    #define ros_debug ROS_INFO
#else
    #define serial_debug(...)
    #define ros_debug(...)
#endif
#define MAX_INFO_BYTES 1024*1024
//#define POSITION					//使用位置式的标签,默认取消，位置式云台会抽搐

class SerialCom{
public: 
    SerialCom();                                    //构造函数，在此打开串口
    ~SerialCom();                                   //析构
    ros::NodeHandle nh;                             //节点管理器
    ros::Subscriber para_sub;                       //接收
    ros::Publisher para_pub;                        //发送云台信息给图像处理节点        
    serial::Serial ser;                             
public:
    /** @brief 从云台板数据中取出数据
     * @param d 外来消息（输入/输出）
     * @return 返回是否读取成功
    */
    int getDataFromSerial(serial_com::comm &d);

    /**
     * @brief 数据转为 uint_8数组（即char数组），以进行云台板数据收发
     * @param buffer 入参/输出:用于储存转换后数据的容器
     * @param x 最优装甲板位置x
     * @param y 最优装甲板位置y
     * @param z 最优装甲板位置z
     * @param status 状态码
    */
    void getBuffer(uint8_t *buffer, float x, float y, float z, uint8_t stat);
    void infoExchange(const serial_com::comm::ConstPtr &msg); //回调函数，负责与云台板进行收发

    /** @brief 从云台板数据中取出pitch,yaw位置
     * @param buffer 云台信息
     * @param yaw yaw值
     * @param pitch pitch值
     * @param ser 装甲板锁定序列
     * @param status 状态码
    */
    void receiveData(const uint8_t *buffer, float &yaw, float &pitch, float &ser, uint8_t &stat);    
private:
    int serialOK(char *output);                                     //查找可用设备，查找到将返回0
    void updateOld(const serial_com::comm &src);                     //更新旧值
    void getFromOld(serial_com::comm &src);                         //从旧值中取数
private:
    serial_com::comm old_val;                   //旧值储存
};
#endif //_SERIAL_COM_HPP
