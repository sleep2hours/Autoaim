#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <iostream>
#include <chrono>
#include "serial_com/comm.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 欧拉角转四元数 RPY 定义
static Eigen::Quaterniond angle2Quat(float r, float p, float y, bool is_rad = false){
    if (is_rad == false){
        r *= 0.0174533;       // pi / 180 = 0.0174533
        p *= 0.0174533;
        y *= 0.0174533;     
    }
    return Eigen::Quaterniond (
        cosf(r / 2) * cosf(p / 2) * cosf(y / 2) + sinf(r / 2) * sinf(p / 2) * sinf(y / 2),  // w
        sinf(r / 2) * cosf(p / 2) * cosf(y / 2) - cosf(r / 2) * sinf(p / 2) * sinf(y / 2),  // x
        cosf(r / 2) * sinf(p / 2) * cosf(y / 2) + sinf(r / 2) * cosf(p / 2) * sinf(y / 2),  // y
        cosf(r / 2) * cosf(p / 2) * sinf(y / 2) - sinf(r / 2) * sinf(p / 2) * cosf(y / 2)   // z
    );
}

class Predict {
public:
    Predict();
    ~Predict();
public:
    /// @brief 输入当前敌方位姿，根据pitch, yaw进行投影，在世界坐标系下进行预测，预测结果反投影回到相机坐标系，便于电控接收
    /// @return true 预测成功,使用预测值,否则使用装甲板
    bool translatePredict(const cv::Point3f& t_cam, const serial_com::comm& msg, Eigen::Vector3d& cam_p);

    /// @brief 切换目标需要进行reset
    void reset();
private:
    /// 根据当前值计算观测值，需要计时
    void calcObvserved(const Eigen::Vector3d& pw, Vector6d& obs, double dt, double lambda = 0.5) const;

    /// 计算状态转移矩阵
    void calcStateTransit(double dt);

    static void project2World(
        const cv::Point3f& t_cam,
        float pit, float yaw,
        Eigen::Vector3d& pw,
        Eigen::Quaterniond& c2w
    );
private:
    bool init;
    Matrix6d A;      // 状态转移
    Matrix6d P;      // 状态转移协方差
    Matrix6d Q;      // 状态转移误差
    Matrix6d R;      // 观测误差
    Vector6d state_post;    // 后验状态
    Vector6d state_pre;     // 先验状态
    std::chrono::_V2::system_clock::time_point saved_time_point;
};