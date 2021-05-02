/**=================AIM_DEPENDENCIES================
 * @author hqy
 * @date 2020.2.5
 * @brief 自瞄多个模块所依赖的结构
 * 最近修改：将会修改Target
 * 
*/

#ifndef AIM_DEPS_CC
#define AIM_DEPS_CC

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#define SENTRYDECISION

enum ROBOT_STATUS
{
    DISTANCE_AIM = 0, //远程自瞄
    AUTO_AIM = 1,     //近程自瞄
    SMALL_BUFF = 2,   //小符打击
    LARGE_BUFF = 3    //大符打击
};

#define Pi 3.1415926
namespace aim_deps
{

    ///============COLOR===============///
    const cv::Scalar RED = cv::Scalar(0, 0, 255);
    const cv::Scalar ORANGE = cv::Scalar(0, 127, 255);
    const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
    const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
    const cv::Scalar CYAN = cv::Scalar(255, 255, 0);
    const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
    const cv::Scalar PINK = cv::Scalar(255, 0, 255);
    const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
    const cv::Scalar GREY = cv::Scalar(127, 127, 127);
    const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
    const cv::Scalar PURPLE = cv::Scalar(255, 10, 100);

    ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
    //==========================通用的预设===========================//
    ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
    //=================最小装甲板面积==============
    const double _HALF_LENGTH_SMALL = 65.00;
    const double _HALF_LENGTH_BIG = 105.00;
    const double _HALF_HEIGHT = 28.50;

    const uint8_t NO_SHOOTING = 64;

    const float MIN_ARMOR_AREA = 40.0;
    const cv::Point2f NULLPOINT2f = cv::Point2f(0.0, 0.0);
    const cv::Point3f NULLPOINT3f = cv::Point3f(0.0, 0.0, 0.0);
    const float RAD2DEG = 57.2958;  //(constant)(180/pi)
    const float DEG2RAD = 0.017453; //(constant)(pi/180)

    //LightMatch.hpp 的依赖参数
    const float LIGHT_PARAM1 = 5;
    const float LIGHT_PARAM2 = 2.4;
    const float LIGHT_mean = 40.0;
    const float FAILED_SCORE = INFINITY;

    enum PLATE_TYPE
    {
        UNKNOWN = 0,
        SMALL = 1,
        LARGE = 2
    };

    //======================相机参数=========================
    const cv::Mat HERO_INTRINSIC = (cv::Mat_<double>(3, 3) << 1776.67168581218, 0, 720,
                                    0, 1778.59375346543, 540,
                                    0, 0, 1);

    const cv::Mat INF_INTRINSIC = (cv::Mat_<double>(3, 3) << 1770.89047225438, 0, 720,
                                   0, 1769.66942390649, 540,
                                   0, 0, 1);

    const std::vector<float> HERO_DIST = std::vector<float>{
        -0.419212525827893,
        0.175006995615751,
        0.00489209817799368,
        -0.00289049464268412};

    const std::vector<float> INF_DIST = std::vector<float>{
        -0.457561248586060,
        0.272138858774427,
        0.00303236038540222,
        0.00225295567632989};

    const cv::Mat NEW_INF_INTRINSIC = (cv::Mat_<double>(3, 3) << 1631.285522460938, 0, 722.7377887028852,
                                       0, 1692.166015625, 541.7929725137074,
                                       0, 0, 1);
    //决策所需参数
    struct PnP_depended_param
    {
        float Distant_base = 0.03;
        float Rotation_base = 30;
        float Size_base = 0.1;
        int Distance_multi = 1;
        int Sentry_score = 1;
        int Hero_score = 1;
        int Infantry_score = 1;
        int Dad_score = 0;
        int Base_score = 10;
        int None_score = 0;
    };
    extern PnP_depended_param pnp_depended_param;

    struct Sentry_decision
    {
        int blood_limit = 50;
        int bullet_limit = 150;
    };
    //装甲板类别
    enum Armor_type
    {
        Sentry,   //哨兵
        Hero,     //英雄
        Infantry, //步兵
        Dad,      //奶3.5爸
        Base,     //基地
        None,     //未知
    };

    ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
    ///=========================通用函数========================///
    ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++///
    /// 返回距离的平方
    inline float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2)
    {
        return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    inline void getTopCenter(const cv::RotatedRect &rect, cv::Point2f &p1, cv::Point2f &p2)
    {
        cv::Point2f tmp_p1, tmp_p2, corners[4]; //找出角点
        rect.points(corners);
        float d1 = getPointDist(corners[0], corners[1]); //0/1点距离的平方
        float d2 = getPointDist(corners[1], corners[2]); //1/2点距离的平方
        int i0 = d1 > d2 ? 1 : 0;                        //长所在边第一个顶点的位置
        tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;    //获得旋转矩形两条短边上的中点
        tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
        if (tmp_p1.y > tmp_p2.y)
        { //保证输出点的顺序
            p2 = (tmp_p1 + tmp_p2) / 2;
            p1 = tmp_p2;
        }
        else
        { //必须是p1是处于上方的点，p2处于下方（y轴更大）
            p1 = tmp_p1;
            p2 = (tmp_p1 + tmp_p2) / 2;
        }
    }

    inline float getLineAngleRad(const cv::Point2f &p1, const cv::Point2f &p2)
    {
        return atan2f(p2.x - p1.x, p2.y - p1.y);
    }

    inline float getLineAngle(const cv::Point2f &p1, const cv::Point2f &p2)
    {
        return getLineAngleRad(p1, p2) * aim_deps::RAD2DEG;
    }

    inline cv::Point2f Rotate(const cv::Point2f &_vec, float _a)
    {
        cv::Point2f res;
        res.x = _vec.x * cos(_a) - _vec.y * sin(_a);
        res.y = _vec.x * sin(_a) + _vec.y * cos(_a);
        return res;
    }

    ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///
    //=======================灯条和装甲板定义========================//
    ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++=///

    struct LightBox
    {
        float length;       // 灯条长度
        float angle;        // 灯条角度
        cv::Point2f vex[2]; // vex[0]在上, vex[1]在下
        cv::Point2f center;

        inline void extend(const float k)
        { // 长度乘以系数变化
            vex[0] += (k - 1.0) * (vex[0] - center);
            vex[1] += (k - 1.0) * (vex[1] - center);
            length *= k;
        }

        inline void rotate(const float ang)
        { // 旋转灯条(依照坐标系逆时针)
            cv::Point2f tmp = vex[1] - center;
            tmp = Rotate(tmp, ang);
            vex[1] = center + tmp;
            vex[0] = center - tmp;
            //printf("Original: %f, rotation: %f\n", angle, RAD2DEG * ang);
            angle -= RAD2DEG * ang; // 注意我们定义的角度是逆时针正，与旋转矩阵定义恰好相反
            while (std::abs(angle - 360) < std::abs(angle))
            {
                angle -= 360;
            }
            while (std::abs(angle + 360) < std::abs(angle))
            {
                angle += 360;
            }
        }
    };

    /// @brief 灯条的两点式线段表示
    struct Light
    {
        bool valid; //是否是有效灯条(反光灯条过滤无法完全精准判定，需要依靠装甲板匹配)
        int index;
        int isLeft = -1; // 用于camera_map优化中，是否为左灯条 0 为左，1为右，否则为未知
        LightBox box;
        Light()
        {
            valid = false;
        }

        Light(
            const cv::RotatedRect &_r,
            const cv::Point2f &_p = NULLPOINT2f,
            float len = 0.0) : valid(false)
        {
            getTopCenter(_r, box.vex[0], box.center);
            box.vex[1] = box.center * 2 - box.vex[0];
            box.length = cv::max(_r.size.height, _r.size.width);
            box.angle = aim_deps::getLineAngle(box.vex[0], box.vex[1]);
        }

        Light(
            const cv::Point2f &tp,
            const cv::Point2f &ctr) : valid(false)
        {
            box.vex[0] = tp;
            box.center = ctr;
            box.vex[1] = 2 * ctr - tp;
            box.length = std::sqrt(aim_deps::getPointDist(box.vex[0], box.vex[1]));
            box.angle = aim_deps::getLineAngle(box.vex[0], box.vex[1]);
        }
    };

    struct Armor
    {
        //可能要删除的valid标签（只需要根据数字判断是否valid就好了）
        bool valid;
        bool Isbigarmor;
        cv::Mat r_vec; //向量
        int armor_number;
        cv::Point3f t_vec;
        cv::Point2f vertex[4];
        Light left_light;
        Light right_light;
        Armor() { valid = true; } //default
        Armor(cv::Point2f _pts[4], int _num, Light _l, Light _r, bool _big = false) : valid(true), Isbigarmor(_big), armor_number(_num), left_light(_l), right_light(_r)
        {
            for (int i = 0; i < 4; ++i)
                vertex[i] = _pts[i]; //copy by points
        }

        /// @brief 存在共灯条,则返回相同的灯条的下标，否则返回-1
        inline int collide(const Armor &a)
        {
            if (left_light.index == a.left_light.index ||
                left_light.index == a.right_light.index)
                return left_light.index;
            if (right_light.index == a.left_light.index ||
                right_light.index == a.right_light.index)
                return right_light.index;
            return -1;
        }
    };

    //决策之后的装甲板
    struct Evaluated_armor
    {
        Armor _armor;
        Armor_type _type;
        float Distance_score;
        float Size_score;
        float Rotation_score;
        float Type_score;
        float Total_score;
    };

    struct Distance_Params
    {
        const float OPS_RATIO_HEIGHT = 25.0; //对边宽比例        (16.0)
        const float OPS_RATIO_WIDTH = 1.69;  //对边长比例        (1.44)
        const float ANGLE_THRESH = 16.0;     //角度差阈值        (13.5)
    };
    extern Distance_Params distance_params;

    //储存检测装甲板的各种参数
    struct Vicinity_param
    {
        //预处理信息
        int brightness_threshold;
        int color_threshold;
        float light_color_detect_extend_ratio;
        cv::Mat blue_lower_bound;
        cv::Mat blue_upper_bound;
        cv::Mat red_lower_bound;
        cv::Mat red_upper_bound;
        //光条本体信息
        float light_min_area;
        float light_max_angle;
        float light_min_size;
        float light_contour_min_solidity;
        float light_max_ratio;

        //光条配对信息
        float light_max_angle_diff_;        //光条最大倾斜角度
        float light_max_height_diff_ratio_; // 光条最大宽高比
        float light_max_y_diff_ratio_;      // 两光条最大y距离比
        float light_min_x_diff_ratio_;      //两光条最大x距离比

        //装甲板信息
        float armor_big_armor_ratio;
        float armor_small_armor_ratio;
        float armor_min_aspect_ratio_; //装甲宽高比
        float armor_max_aspect_ratio_;
        int enemy_color; //目标颜色
                         //
        float sight_offset_normalized_base;
        float area_normalized_base;
        //构造函数
        Vicinity_param()
        {
        }
        Vicinity_param(bool isblue, int h_min, int s_min, int v_min, int h_max, int s_max, int v_max) //int Bright_threshold,int Color_threshold)
        {
            if (isblue)
            {
                blue_lower_bound = (cv::Mat_<int>(1, 3) << h_min, s_min, v_min);
                blue_upper_bound = (cv::Mat_<int>(1, 3) << h_max, s_max, v_max);
            }
            else
            {
                red_lower_bound = cv::Mat_<int>(h_min, s_min, v_min);
                red_upper_bound = cv::Mat_<int>(h_max, s_max, v_max);
            }

            brightness_threshold = 150;
            color_threshold = 100;
            light_color_detect_extend_ratio = 1.1;
            light_min_area = 10;
            light_max_angle = 45.0;
            light_min_size = 5.0;
            light_contour_min_solidity = 0.5;
            light_max_ratio = 1.0;
            light_max_angle_diff_ = 7.0;
            light_max_height_diff_ratio_ = 0.2;
            light_max_y_diff_ratio_ = 2.0;
            light_min_x_diff_ratio_ = 0.8;
            armor_big_armor_ratio = 3.2;
            armor_small_armor_ratio = 2;
            armor_min_aspect_ratio_ = 1.0;
            armor_max_aspect_ratio_ = 5.0;
            sight_offset_normalized_base = 200;
            area_normalized_base = 1000;
        }
    };
    struct WindMill_param
    {
        int S_R_min = 800;
        int S_R_max = 1300;
        int S_Leaf_min = 3300;
        int S_Leaf_max = 4200;
        float Ratio_min = 0.75;
        float Ratio_max = 1.15;
        int R = 170;         //能量机关图像半径
        float length = 0.68; //能量机关实际宽度
        int Dist_min = 105;
        int Dist_max = 140;
        int hight = 900;       //云台标准位置离R高度
        float hit_dx = 7.1737; //打击点距能量机关R水平距离
        float offset_x = 0;    //枪口和摄像头偏移补偿量
        float offset_y = -47;
        float offset_z = 119.2;
        double constant_speed = 60.0;
        bool direction = true; //是否逆时针转动
        float init_k_ = 0.02;
    };
} //namespace aim_deps
#endif //AIM_DEPS_CC
