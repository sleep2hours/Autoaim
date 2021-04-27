#ifndef WINDMILL_HPP
#define WINDMILL_HPP

#include "WM_Param.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "serial_com/comm.h"
#include <vector>
#include <deque>
#include "aim_deps/AimDeps.hpp"

#define debug_show
// #define debug_show_fins
// #define debug_show_armors
// #define debug_show_R
namespace WINDMILL
{
    class windMill
    {
    public:
        windMill(bool blue);
        ~windMill();
        void Hit(cv::Mat &img, serial_com::comm &msg, int present); //二营长的意大利炮
        void reset();
        double last_t = 0.0;
        double now_t = 0.0;
        double start_t = 0.0; //采样初始时间
    private:
        bool FindHitPoint(cv::Mat &img);                                                                            //找到图像打击点
        bool GetContours(cv::Mat &img);                                                                             //获取装甲版,叶片,R
        bool FindValidFinArmor(std::vector<std::vector<cv::Point>> fin_contours, std::vector<cv::Vec4i> hierarchy); //筛选有效装甲板及叶片
        bool FindValidR(std::vector<std::vector<cv::Point>> R_contours);                                            //筛选有效R
        bool IsValidFin(std::vector<cv::Point> contour);                                                            //判断是否为有效叶片
        bool IsValidArmor(std::vector<cv::Point> contour);                                                          //判断是否为有效装甲板
        bool IsValidR(std::vector<cv::Point> contour);                                                              //判断是否为有效R
        void DirectionJudge();                                                                                      //判断转动方向
        void SmallPredict(float v, float pitch_deg);                                                                //小符预测
        void BigPredict(float v, float pitch_deg);                                                                  //大符预测 正弦速度表达式:：spd=0.785*sin（1.884*t)+1.305
        void FaiFit();                                                                                              //拟合速度正弦表达式
        void clear();

    private:
        cv::Mat src;
        wm_param params;  //风车参数
        velocity_fun spd; //正弦表达式参数；
        int mode;         //当前模式
        bool _isblue;     //敌方是否是蓝色
        float last_angle = 0.0;
        float now_angle = 0.0;
        float pre_angle = 0.0;
        float dangle_rad = 0.0;
        float dangle_deg = 0.0;
        cv::Point2f center; //大符中心R
        cv::Point2f hit_point;
        cv::Point2f pre_point;
        cv::Point2f fin_vec;                              //叶片中心指向装甲板中心的向量
        std::vector<std::vector<cv::Point>> fin_contours; //所有可能叶片及装甲板
        std::vector<std::vector<cv::Point>> R_contours;   //所有可能R
        std::vector<cv::RotatedRect> armors;              //装甲版容器
        std::vector<cv::RotatedRect> fins;                //叶片容器
        std::vector<cv::RotatedRect> center_pt;           //大符中心预选容器
        bool direction = true;                            //风车旋转方向:是否顺时针旋转
        int clockwize;                                    //顺时针转动数
        int anticlockwize;                                //逆时针转动数
        int cnt;                                          //处理帧数
        std::deque<double> t_list;
        std::deque<double> anglevelocity_rad; //风车旋转速度

    public:
        inline double DistCal(cv::Point a, cv::Point b) //计算两点距离
        {
            double dist;
            double dx = a.x - b.x, dy = a.y - b.y;
            dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            return dist;
        }

        inline cv::Point CalPoint(cv::Point2f center, double R, double angle_deg) //根据角度和中心算点
        {
            return center + cv::Point2f((float)cos(angle_deg / 180 * Pi), (float)-sin(angle_deg / 180 * Pi)) * (float)R;
        }

        inline double CalAngle(cv::Point p1, cv::Point center) //根据中心和点算角度
        {
            cv::Point2f dp = p1 - center;
            dp.y = -dp.y;
            if (dp.x == 0)
            {
                if (dp.y >= 0)
                    return 0;
                else
                    return 180;
            }
            float angle = atan(dp.y / dp.x) / Pi * 180;
            if (dp.x >= 0 && dp.y > 0)
            {
                return angle;
            }
            else if (dp.y > 0 && dp.x < 0)
            {
                return 180 + angle;
            }
            else if (dp.y <= 0 && dp.x < 0)
            {
                return 180 + angle;
            }
            else if (dp.y < 0 && dp.x > 0)
            {
                return 360 + angle;
            }
        }

        inline double HitDist(double angle_deg) //计算打击点到相机的距离的水平投影
        {
            double dx = params.length * cos(angle_deg / 180 * Pi);
            double dist = std::sqrt(dx * dx + params.hit_dx * params.hit_dx);
            return dist;
        }

        inline double SPreNextAngle(double angle_deg, double dt) //计算一定时间后匀速风车的角度
        {
            double next_angle = angle_deg + params.constant_speed * dt;
            if (next_angle < 0)
            {
                next_angle = 360 + next_angle;
            }
            if (next_angle > 360)
            {
                next_angle -= 360;
            }
            return next_angle;
        }
        inline double BPreNextAngle(double angle_deg, double t0, double dt) //计算一定时间后正弦风车的角度
        {
            double angle_sum = spd.A0 * dt + (spd.A / spd.w) * (cos(spd.w * t0 + spd.fai) - cos(spd.w * (t0 + dt) + spd.fai));
            if (direction == true)
                angle_sum *= -1;
            double next_angle = angle_deg + angle_sum ;
            // if (next_angle < 0)
            // {
            //     next_angle = 360 + next_angle;
            // }
            // if (next_angle > 360)
            // {
            //     next_angle -= 360;
            // }
            return next_angle;
        }
        inline double CalTime(double x, double v, double angle_deg) const
        {
            return (double)(-1 / (params.init_k_) * log(1 - params.init_k_ * x / (v * cos(angle_deg / 180 * Pi))));
        }
        inline double CalVecAngle(cv::Point2f cen, cv::Point2f fin_vec) //计算两向量夹角
        {
            return acos((cen.x * fin_vec.x + cen.y * fin_vec.y) / (sqrt(cen.x * cen.x + cen.y * cen.y)) / (sqrt(fin_vec.x * fin_vec.x + fin_vec.y * fin_vec.y)));
        }
        double FaiDiff(double fai, std::deque<double> t_list, std::deque<double> anglevelocity_rad) //计算正弦拟合函数一阶导
        {
            double diff1 = 0;
            for (int i = 0; i < t_list.size(); i++)
            {
                diff1 += (spd.A * sin(spd.w * t_list[i] + fai) + spd.A0 - anglevelocity_rad[i]) * spd.A * cos(spd.w * t_list[i] + fai);
            }
            return diff1;
        }
        double FaiDiff2(double fai, std::deque<double> t_list, std::deque<double> anglevelocity_rad) //计算正弦拟合函数二阶导
        {
            double diff2 = 0;
            for (int i = 0; i < t_list.size(); i++)
            {
                diff2 += (spd.A * spd.A * std::pow(cos(spd.w * t_list[i] + fai), 2) - (spd.A * sin(spd.w * t_list[i] + fai) * (spd.A * sin(spd.w * t_list[i] + fai) + spd.A0 - anglevelocity_rad[i])));
            }
            return diff2;
        }
    };
}
#endif