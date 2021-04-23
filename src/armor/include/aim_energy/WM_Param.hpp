#ifndef WM_PARAM_HPP
#define WM_PARAM_HPP

#define small_buff 2
#define big_buff 3
#define Pi 3.1415926

namespace WINDMILL
{

    const int exp_time = 4000;

    struct wm_param
    {
        int s_armor_min = 1100;
        int s_armor_max = 1900;
        float armor_ratio_min = 1.2;
        float armor_ratio_max = 2.0;
        float s_armor_ratio_min = 0.7;
        float s_armor_ratio_max = 1;

        int s_R_min = 80;
        int s_R_max = 12400;
        float R_ratio_min = 0.95;
        float R_ratio_max = 1.4;
        float s_R_ratio_min = 0.69;
        float s_R_ratio_max = 1;

        int s_leaf_min = 3900;
        int s_leaf_max = 5600;
        float fin_ratio_min = 1.6;
        float fin_ratio_max = 2.4;
        float s_fin_ratio_min = 0.3;
        float s_fin_ratio_max = 0.5;

        int R = 170;         //能量机关图像半径
        float length = 0.68; //能量机关实际宽度
        int dist_min = 105;
        int dist_max = 140;
        int hight = 900;       //云台标准位置离R高度
        float hit_dx = 7.1737; //打击点距能量机关R水平距离
        float offset_x = 0;    //枪口和摄像头偏移补偿量
        float offset_y = -47;
        float offset_z = 119.2;
        double constant_speed =45;
        bool direction = true; //是否逆时针转动
        float init_k_ = 0.02;
    };
}

#endif