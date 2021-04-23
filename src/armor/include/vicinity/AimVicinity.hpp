#ifndef AimVicinity_HPP
#define AimVicinity_HPP 
#include <map> 
#include <thread>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Detective.hpp"
#include "../aim_deps/AimBase.hpp"
//#include "../anti_top/AntiTop.hpp"
#include "../../../serial_com/include/serial_com/comm.h"
#define MODE_SPECIFY                                //如果不打印显示自瞄内部的模式，请注释此行
#define AIM_VIC_DEBUG                               //如果不进行debug输出,请注释此行
#ifdef AIM_VIC_DEBUG
    #define vic_out rmlog::LOG::printc
#else
    #define vic_out(...)
#endif 

namespace AimVicinity
{

class AimVicinity: public aim_base::AimBase{
public:
    AimVicinity(bool _blue, int h_min,int s_min,int v_min,int h_max,int s_max,int v_max);                  //参数_blue意为 (enemy_is)_blue
    ~AimVicinity();
public:
    /**
     * @brief 近距离打击一条龙集成处理
     * @param src 由取流解码得到的图像，必要的时候还会在图像上进行绘制
     * @return 云台状态信息
    */
    int aimAuto(cv::Mat &src, serial_com::comm &msg);     //一条龙服务
private:
    //===================自瞄内部基本模式函数====================//
    void freeTarget(cv::Mat &src, serial_com::comm &msg);       //FREE_TARGET自由瞄准
    //=======================================================//
    void aimRecognition(const cv::Mat src);                 //对所有待选装甲板进行数字识别
    void aimDisplay(cv::Mat &src, const serial_com::comm msg);//信息显示在屏幕上（debug功能）            
private:
    aim_deps::Vicinity_param _param;
    Detective detecting_module;
};
}   //namespace AimVicinity
#endif  //AimVicinity_HPP