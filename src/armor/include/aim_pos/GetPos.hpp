/**==========pnp测距测试模块，基于opencv solvePNP的iterative算法==============
 * @author: sentinel
 * last date of modification: 2020.3.2
 * 最后修改的内容：删除了几个没有用的函数和变量
*/

#ifndef _GET_POS_HPP
#define _GET_POS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include "../aim_deps/AimDeps.hpp"	
#include "../../../serial_com/include/serial_com/comm.h"
//#include "GimbalCtrl.hpp"
#define DO_STATIC_FILTER        // GetPos       PnP测距模块         静止目标滤波

//#define GIMBAL_DEBUG_PRINT                // 弹道模型（即将删除）
#ifdef GIMBAL_DEBUG_PRINT
    #define gimbal_debug printf
#else
    #define gimbal_debug(...)
#endif //GIMBAL_DEBUG_PRINT

class GetPos{
public:
	GetPos();
	~GetPos();

	/**
	 * @brief 将距离解算与弹道模型，角度预测分开进行
	 * @param pos 最终的预选装甲板
	 * @param msg 云台信息交互载体
	 */
	void calcBallistic(cv::Point3f &pos, serial_com::comm &msg);	
	//ratio:角度缩小控制率
	//比如角度过大，在发一次信息过程中转不完计算的角度，可以认为缩小每次发送的角度
	void batchProcess(std::vector<aim_deps::Armor> &tar_list) const;	//批量solvePNP（但是不解算弹道）
	void positionScore(aim_deps::Armor &tar) const;		//距离分数和旋转分数计算	

	/**
	 * @brief 从tar_list中获取rMats与tMats
	 * @param rmats rMats
	 * @param tmats tMats
	 * @param tar_list 通过其中的Armor的属性解出旋转矩阵以及取出平移向量
	 */
	void packUp(std::vector<cv::Mat>& rmats, std::vector<cv::Mat> &tmats, 
		const std::vector<aim_deps::Armor>& tar_list) const;	

	bool doSlideWindow(cv::Point3f pos, cv::Point2f gimbal, int order = 9);

	// 相关参数重置
	void reset(){
        idle_cnt = 0;
        filter.clear();
        gimpos.clear();
        mean_pos = cv::Point3f(0.0, 0.0, 0.0);
        mean_gim = cv::Point2f(0.0, 0.0);
    }
private:
	// 保持队列长度为max_size的push_back操作
    template<typename Ty = cv::Point2f>
    static void push_back(std::deque<Ty> &de, Ty num, size_t max_size = 12){
        if(de.size() >= max_size){
            de.pop_front();
            de.emplace_back(num);
        }
        else{
            de.emplace_back(num);
        }
    }  
public:
	cv::Mat tVec;										//平移矩阵(x, y, z)
	// ballistic::GimbalCtrl g_ctrl;					//弹道模型
private:
	std::vector<cv::Point3f> objPoints_small;			//世界坐标系下的点（装甲板实际4点位置）
	std::vector<cv::Point3f> objPoints_big;				//世界坐标系下的点（装甲板实际4点位置）
	std::vector<float> distCoeffs;						//畸变与切变向量（4参数）
	cv::Mat rVec;										//旋转向量
	cv::Mat insM;										//内参矩阵Mat形式

	int idle_cnt;                       				// 敌方静止的帧数

    cv::Point2f mean_gim;               				// 平均云台位置

    cv::Point3f mean_pos;               				// 敌方目标静止时的平均位置
    std::deque<cv::Point3f> filter;     				// 静止目标滑动平均(7)
    std::deque<cv::Point2f> gimpos;           			// 云台pitch yaw角度和队列(8)
};
#endif //_GET_POS_HPP