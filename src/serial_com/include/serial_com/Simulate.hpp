#ifndef _SIMULATE_HPP
#define _SIMULATE_HPP

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include "comm.h"
#include <opencv2/core.hpp>
//#define DO_PRINTF
#ifdef DO_PRINTF
    #define print printf
#else
    #define print(...)
#endif

namespace sim{

const float PITCH_VEL = 0.5;
const float YAW_VEL = 1.0;
const float start_pitch = -15.0;
const float start_yaw = -20.0;

enum MODE{
    NOISE_LINEAR,           //线性有噪声
    RANDOM,                 //单点附近随机分布
    STATIC,                 //静态
    FOLLOW                  //跟随
};

class Simulate{
public:
    Simulate(ros::NodeHandle nh, int _mode = 0, int is_blue = false);
    ~Simulate();
public:
    void display(const serial_com::comm::ConstPtr &msg);            //显示从自瞄进程来的消息
public:
    ros::Publisher pub;
    ros::Subscriber sub;
    bool log_judge;
private:
    //generator是包含在 display这个回调函数里的，所谓接受完就发送
    void generator(MODE mode, float _pit = 0.0, float _yaw = 0.0);        //云台信息生成
    void decoder(const uint8_t *buffer, serial_com::comm &msg) const;     //解码(从serialcom复制来的)

    /**
     * @brief 数据转为 uint_8数组（即char数组），以进行云台板数据收发
     * @param buffer 入参/输出:用于储存转换后数据的容器
     * @param x 最优装甲板位置x
     * @param y 最优装甲板位置y
     * @param z 最优装甲板位置z
     * @param status 状态码
    */
    void encoder(uint8_t *buffer, float x, float y, float z, uint8_t stat) const;

    /**TODO: GimbalCtrl以及GetPos还没有bullet_vel以及x_num, y_num的接口*/
    ///需要随机生成的只有：pitch/yaw/x_num/y_num/
    void linear(float &pitch, float &yaw);
    void onePoint(float &pitch, float &yaw);
    void follow(float &pitch, float &yaw);
    void staticPoint(float &pitch, float &yaw);
    float random(int range = 2);                                //默认随机值产生范围是(-2, 2)
    float serSwitch();                                   //通过现在的frmae_cnt调节ser的输出
private:
    int mode_cnt;                       //模式计数器（每次变换，会导致status从1-0变化）
    int frame_cnt;                      //帧数计数器
    int seed;                           //随机数生成seed
    int _sign_p;                        //pitch轴符号
    int _sign_y;                        //yaw周符号
    int _is_blue;                       //敌人是否为蓝色
    float linear_pitch;
    float linear_yaw;
    float old_pitch;                    
    float old_yaw;
    cv::RNG rng;                        //随机数发生器                               
};          
}   //namespace sim

#endif //_SIMULATE_HPP