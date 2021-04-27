/**
 * ====================Aim 基类=====================
 * 在中近距离和远距离打击中，有过多的函数重合（甚至没有改动）
 * 默认的继承方式是public(protected应该也可，不可private)
 * 作者：hqy
 * 最后修改日期: 2020.5.19
 * 简介：
 *      AimBase与aimDistance将直接继承这个类，这个类中所有函数都可调用
 *      需要提的事情是，派生类构造函数中还需要加入派生类需要的语句，默认构造不包括
 */
#ifndef _AIM_BASE_HPP
#define _AIM_BASE_HPP 
#include <map> 
#include <thread>
#include <vector>
#include <iostream>
#include <numeric>
#include "AimDeps.hpp"
#include "aim_rec/NumRec.hpp"
#include "aim_pos/GetPos.hpp"
#include "aim_pre/Predict.hpp"
#include "aim_dec/Armordecision.hpp"
#include "serial_com/comm.h"
// #define DISPLAY_TARGETS         // AimBase      自瞄基类            显示处理后的有目标图像
// #define BASE_CNT_TIME        // AimBase      自瞄基类            自瞄基类时间测试
// #define MODE_SPECIFY         // AimBase      自瞄基类            显示自瞄内部的模式

namespace aim_base{

const int NULLPOS = -1;
const int debug_delay = 60;

//远近瞄准使用的基类
class AimBase{
public:
    AimBase();
    ~AimBase();
    void reset(){                               //切换模式，本模式所有的遗留信息都要清除
        _x = 0.0;
        _y = 0.0;   
        _z = 0.0;
        now_delay = 0;                          // TODO: 需要删除
        optimal         = NULLPOS;
        last_match      = 0;
        pos_getter.reset();              // 滑动平均滤波器重置    
    }                                           

    // 弹道模型从参数服务器取回设置值

protected:
    bool seqMatch(cv::Mat &src, serial_com::comm &msg);         //SEQ_MATCH序列匹配
    void aimPosition();                                         //位姿信息解算

    /// TODO: 此处为雷达的预测？
    void aimStrategy(){;}                                   

    /**
     * @brief 计算弹道模型：now_delay在这个地方取得，同时也在此处判断是否发弹
     * @param msg 把弹道相关的角度写入msg中
     * @param pos 指定对pos位的tar_list进行计算,如果为NULLPOS，则对成员变量(int)optimal位解算
     * @note idle模式以及seqmatch模式不发弹在此处判断,预测精准性对发弹的影响判断也在此处
     */
    void aimBallistic(serial_com::comm &msg);       //弹道模型解算（只解算最优装甲板）

    #ifdef SENTRYDECISION
    int Switch_mode();
    #endif //SENTRYDECISION
protected:
    PnPdecision aim_dec; 
    GetPos pos_getter;
    NumRec classifier;                                          //数字分类器
    // Predict aimp;
protected:
    bool _is_enemy_blue;
    //===================//
    int optimal;                                            // 最优装甲板的下标（tar_list中的位置）

    float _x;
    float _y;
    float _z;
    float last_match;                                       // 上次匹配数字
    float now_delay;                                        // 临时使用

    std::vector<aim_deps::Armor> tar_list;                  // 多模块共用的装甲板预选容器

    #ifdef SENTRYDECISION
        std::vector<aim_deps::Armor> Last_10_targets; 
    #endif  //SENTRYDECISION

    #ifdef BASE_CNT_TIME
        double sum_t;
        double _cnt;
    #endif // BASE_CNT_TIME
};
}       //namespace aim_base

#endif //_AIM_BASE_HPP