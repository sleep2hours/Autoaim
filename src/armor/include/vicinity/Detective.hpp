#ifndef __DETECTIVE_HPP
#define __DETECTIVE_HPP

#pragma once
#include <opencv2/opencv.hpp>
#include <array>
#include <opencv2/ml.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include "opencv2/imgproc/imgproc.hpp"
#include "../aim_deps/AimDeps.hpp"

namespace AimVicinity
{

enum ColorChannels
{
	BLUE = 0,
	GREEN = 1,
	RED = 2
};
enum Armortype
{
	Unknow,
	Big,
	small
};


/*
*   @brief:储存/描述/处理所有的光条
*/
class Lightbar
{
public:
	Lightbar(){};
	Lightbar(const cv::RotatedRect &light)
	{
		width = light.size.width;
		length = light.size.height;
		center = light.center;
		angle = light.angle;
		area = light.size.area();
	}
	const Lightbar &operator=(const Lightbar &ld)
	{
		this->width = ld.width;
		this->length = ld.length;
		this->center = ld.center;
		this->angle = ld.angle;
		this->area = ld.area;
		return *this;
	}

	//将所有光条以rect储存起来
	cv::RotatedRect rec() const
	{
		return cv::RotatedRect(center, cv::Size2f(width, length), angle);
	}

public:
	float width;
	float length;
	cv::Point2f center;
	float angle;
	float area;
};

//包括装甲板信息
class ArmorDetails
{
public:
	//创建一个空的装甲
	ArmorDetails();
	//创建一个包括左光条组，右光条组，原图，装甲参数
	ArmorDetails(const Lightbar &lLight, const Lightbar &rLight,const int Armortype);


public:
	std::array<cv::RotatedRect, 2> lightPairs; //0左1右
	std::vector<cv::Point2f> vertex;		   //装甲轮廓
	int type;
};
struct Light
{
    float width;
	float length;
	cv::Point2f center;
};


/*
*	找一找
*/
class Detective
{
public:
	Detective();
	Detective(const aim_deps::Vicinity_param &vicinity_param);
	~Detective() {}
	//初始化，传入各种参数
	void init(const aim_deps::Vicinity_param &vicinity_param);
	//设定敌方颜色
	void setEnemyColor(int enemy_color)
	{
		_enemy_color = enemy_color;
		_self_color = enemy_color == BLUE ? RED : BLUE;
	}
	//将单帧图片引入
	void loadImg(const cv::Mat &srcImg);
	//检测的主要函数
	int detect();
	//获得装甲轮廓
	//const std::vector<cv::Point2f> getArmorVertex() const;
	//瞄准！！这一部分可以忽略
	//输出图片
	void showDebugImg() const;
	//要发送的数据
	//有各种轮廓的图
			   //预处理后的图片
	const std::vector<aim_deps::Armor> get_all_armors() const;
	cv::Mat get_gray_img();
	void drawArmorPlates( cv::Mat &src, 
				 const std::vector<aim_deps::Armor> tar_list, const int optimal);
public:
	int _enemy_color;					//修改
private:
	aim_deps::Vicinity_param _param;				//
	int _self_color;
	bool found = false;
	cv::Rect _roi;
	cv::Mat _srcImg; //原图
	cv::Mat _roiImg;
	cv::Mat _debugImg;
	cv::Mat _grayImg;	
    std::vector<ArmorDetails> _armors; //所有装甲
	ArmorDetails _targetArmor;		   //目标
#ifdef GET_ARMOR_PIC
	int _allCnt = 0;
#endif // GET_ARMOR_PIC
	int number;

	std::string _debugWindowName = "a";

};
} // AimVicinity
#endif