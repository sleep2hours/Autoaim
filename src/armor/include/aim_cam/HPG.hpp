/*==============ROS高性能取流节点(c++ std::thread实现)================
作者：hqy
创建日期：2020.1.22 10:39
最后修改：2020.5.1 20:49
主要内容：对HPGrabbing节点使用的全局函数的封装
    1.双线程同时取流并且解码，解码后直接进行图像处理（解码和处理是同一线程）
    2.现在估计的取流会比解码更慢，故解码位置是由取流位置决定的
    3.目前帧数最大值：88fps
    4.mutex实现已经尝试，某些地方的处理不够方便，并且效率无区别，故不使用
*/

#ifndef _HP_GRABBING_HPP 
#define _HP_GRABBING_HPP 

#include <atomic>
#include <thread>
#include <iostream>
#include <opencv2/core.hpp>
#include "CameraCtl.hpp"
#include "aim_deps/AimDeps.hpp"

// #define HPG_DEBUG
#ifndef HPG_DEBUG
    #define hpg_debug(...)                                
#else
    #define hpg_debug printf
#endif

namespace hpg{
const int NULL_POS = -1;
const int CYCLE_TIMES = 3;          //高性能取流明暗曝光交替周期(帧数)
const int GRAB_OK = 0, GRAB_FAILED = 1;

enum MODE{
    FROM_CAM                = 0,           //高性能交替曝光
    FROM_VIDEO              = 1,           //从视频取流
    FROM_CAM_LOW_PERFORM    = 2,           //低性能
};

struct ImageBuffer{
    volatile std::atomic<bool> r_busy;     
    volatile std::atomic<bool> w_busy;          
    MV_FRAME_OUT frame;                 //帧数据
};

class HPGrabbing{
public:
    HPGrabbing();
    ~HPGrabbing();

    /** @brief 初始化高性能取流模块
     * @param bb 蓝色白平衡
     * @param rb 红色白平衡
     * @param mode 是否从摄像机进行高性能取流
     * @param path 如果使用视频，视频的路径
     */
    void init(int bb, int rb, int mode, std::string path);

    int getMat(cv::Mat &src);   //src入参/输出, high_exp:是否是高曝光

    void start();                                           //外部开启并detach进程

    /// 对应了Frame中的ROBOT_STATUS这个enum中的值
    /// 设置相机参数 (以及敌方颜色信息(颜色不同参数不同))
    void setCameraParams(int mode);

    inline void setBufferState(){
        buf[last_pos].r_busy = true;                               //一个位置只能解码一次
        buf[last_pos].w_busy = false;          
        hpg_debug("Reading succeeded. [%d].r_busy, w_busy=(%d, %d)\n",
            last_pos, buf[last_pos].r_busy, buf[last_pos].w_busy);
    }
public:
    bool grab;                                              //是否进行grab的标签
    cv::VideoCapture cap;                                   //对视频播放进行集成
private:
    /** 
     * @brief 写入操作线程主循环，
     * @param pUser CameraCtl中的相机操作句柄（handle）
    */
    void* writeWorkThread(void *pUser);  
    bool isWritable(const int pos);                         //bool:pos是否可写
    void readBuffer(const int pos, cv::Mat &src); //对pos进行写入操作，包括解码和图像处理

    /**
     * @brief 对缓冲区的pos位置进行写入操作
     * @param pUser 相机控制模块中的handle
     * @param pos 缓冲区下标，取值为0或1（双缓冲）
     * @param e 入参/输出-SDK错误代码
     * @return 下一个写入的缓冲区下标
    */
    int writeBuffer(int pos, int &e);
private:
    int next_pos;                      //下一次读取的位置
    int last_pos;                      //上一次读取的位置
    int grab_mode;                                  //取流方式from_cam还是from_video
    int exp_short;                                   //长曝光时间(或是非交替曝光的曝光时间)   
    int exp_long;                                   //长曝光时间(或是非交替曝光的曝光时间)  
    int balance_r;
    int balance_b; 
    cm::CameraCtl ctl;      
    ImageBuffer buf[2];                             //乒乓缓冲池
};
}       //namespace hpg
#endif  //_HP_GRABBING_HPP