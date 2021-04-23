#ifndef _AIM_DISTANCE_HPP
#define _AIM_DISTANCE_HPP 
#include <thread>
#include <vector>
#include <iostream>
#include "ArmorPlate.hpp"
#include "LightMatch.hpp"
#include "aim_deps/AimBase.hpp"
#include "serial_com/comm.h"
//#define MODE_SPECIFY                             //如果不打印显示自瞄内部的模式，请注释此行

namespace aim_dist{

//===========远距离攻击类================
class AimDistance: public aim_base::AimBase{
public:
    AimDistance();                   //默认敌人为蓝色
    ~AimDistance();
public:
    /** @brief 初始化aimDistance 详细参数见AimDeps Light_Param */
    void init(bool _blue, int thresh_low, int ch_diff, int filter);

    /**
     * @brief 远距离打击一条龙集成处理
     * @param src 由取流解码得到的图像，必要的时候还会在图像上进行绘制
     * @param msg 来自云台的信息(并写入作为输出)
    */
    void aimAuto(cv::Mat &src, serial_com::comm &msg);       //一条龙服务
public:
    ArmorPlate amp;
    LightMatch match;
private:
    //===================自瞄内部基本模式函数====================//
    void freeTarget(cv::Mat &src, serial_com::comm &msg);       //FREE_TARGET自由瞄准
    //=======================================================//
    void aimRecognition(const cv::Mat &src);                 //对所有待选装甲板进行数字识别
    void aimDisplay(cv::Mat &src, const serial_com::comm &msg);//信息显示在屏幕上（debug功能）  

    /// TODO:需要删除的debug输出
    void aimDebugDisplay(cv::Mat &src, const serial_com::comm &msg);
private:
    Eigen::Vector3d camera_p;
    bool _is_reset;
#ifdef DISPLAY_TARGETS
    char key;
    int _delay;                 //按下e键之后，会以_delay ms每帧的方式进行播放(并修改slow_judge)
    bool slow_judge;            //为true时，会以600ms每帧的方式播放
#endif //DISPLAY_TARGETS
};
}//namespace aim_dist
#endif //_AIM_DISTANCE_HPP
