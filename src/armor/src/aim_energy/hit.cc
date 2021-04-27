#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    void windMill::Hit(cv::Mat &img, serial_com::comm &msg, int present)
    {
        src = img.clone();
        float yaw_deg = msg.x;
        float pitch_deg = msg.y;
        float yaw_rad = msg.x / 180 * Pi;
        float pitch_rad = msg.y / 180 * Pi;
        float v_ball = msg.z;
        mode = present;
        bool find = FindHitPoint(img);
        // std::cout << cnt << std::endl;
        if (find)
        {
            cnt++;
            if (cnt < 25)
            {
                DirectionJudge();
                msg.x = msg.y = msg.z = 0;
            }
            else
            {
                if (mode == small_buff)
                {
                    SmallPredict(v_ball, pitch_deg);
                }
                else if (mode == big_buff)
                {
                    BigPredict(v_ball, pitch_deg);
                }
                float X_direct = (pre_point.x - aim_deps::INF_INTRINSIC.at<double>(0, 2)) / aim_deps::INF_INTRINSIC.at<double>(0, 0);
                float Y_direct = (pre_point.y - aim_deps::INF_INTRINSIC.at<double>(1, 2)) / aim_deps::INF_INTRINSIC.at<double>(1, 1);
                float Z_direct = 1000 * params.hit_dx / (cos(pitch_rad) * cos(yaw_rad) - Y_direct * sin(pitch_rad) - X_direct * cos(pitch_rad) * sin(yaw_rad));
                msg.x = X_direct * Z_direct;
                msg.y = Y_direct * Z_direct;
                msg.z = Z_direct;
            }
        }
        else
        {
            msg.x = msg.y = msg.z = 0;
        }
#ifdef debug_show
        cv::circle(src, hit_point, 5, cv::Scalar(122, 40, 230), -1);

        char str[20];
        snprintf(str, 20, "Pitch:%f", pitch_deg);
        cv::putText(src, str, cv::Point2f(80, 40),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "Yaw:%f", yaw_deg);
        cv::putText(src, str, cv::Point2f(80, 70),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "V_ball:%f", v_ball);
        cv::putText(src, str, cv::Point2f(80, 100),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "dangle_rad:%f", dangle_rad);
        cv::putText(src, str, cv::Point2f(80, 120),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "Angle:%f", now_angle);
        cv::putText(src, str, cv::Point2f(80, 150),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "X:%f", msg.x);
        cv::putText(src, str, cv::Point2f(80, 180),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "Y:%f", msg.y);
        cv::putText(src, str, cv::Point2f(80, 210),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "Z:%f", msg.z);
        cv::putText(src, str, cv::Point2f(80, 240),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
        cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));

        imshow("debug", src);
        cv::waitKey(1);
        if (cv::waitKey(1) == 'e')
        {
            cv::waitKey(0);
        }
#endif
        clear();
    }
}