/**==================视觉组框架==================
 * @build & maintainer hqy
 * @date of modification: 2021/3/18
*/
#include <cstdio>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "readParam.hpp"
#include "aim_cam/HPG.hpp"
#include "aim_deps/AimRos.hpp"
#include "distance/AimDistance.hpp"
#include "vicinity/AimVicinity.hpp"
#include "serial_com/comm.h"
#include "aim_energy/WindMill.hpp"
// #define ORIGINAL_PIC            // Frame        主函数              显示相机原图

int main(int argc, char **argv)
{
    cv::setNumThreads(2);
    ros::init(argc, argv, "main_frame");
    ros::NodeHandle nh;
    ros::Publisher gimbal_pub;

    cv::VideoCapture writer;
    cv::Size size = cv::Size(1440, 1080);
    // writer.open("/home/xjturm/red.avi",writer.four,50,size,true);
    int GRAB_MODE = readParam<int>(nh, "/from_video"),                                // 取流方式
        CURRENT_STATUS = readParam<int>(nh, "/debug_mode"),                           // 当前状态
        OLD_STATUS = CURRENT_STATUS;                                                  // 用于记录上一次循环结束时的状态
    bool init_color_blue = readParam<int>(nh, "/init_color_blue") > 0 ? true : false; //从参数服务器读取

    AimRos aros;
    ros::Subscriber gimbal_sub = nh.subscribe("gimbalData", 1, &AimRos::subCallback, &aros);

    hpg::HPGrabbing grab;

    std::string video_path = readParam<std::string>(nh, "/video_path");
    int balance_r = init_color_blue ? readParam<int>(nh, "/blue_r_balance") : readParam<int>(nh, "/red_r_balance");
    int balance_b = init_color_blue ? readParam<int>(nh, "/blue_b_balance") : readParam<int>(nh, "/red_b_balance");
    cv::Mat frame;
    WINDMILL::windMill wm(init_color_blue);
    gimbal_pub = nh.advertise<serial_com::comm>("cameraData", 1);
#ifdef ORIGINAL_PIC
    cv::namedWindow("origin", cv::WINDOW_AUTOSIZE);
#endif //ORIGINAL_PIC

    grab.init(balance_b, balance_r, GRAB_MODE, video_path); // 高性能取流初始化
    grab.start();
    grab.setCameraParams(CURRENT_STATUS);

    AimVicinity::AimVicinity v_aim(init_color_blue,
                                   init_color_blue ? readParam<int>(nh, "/blue_y_min") : readParam<int>(nh, "/red_h_min"),
                                   init_color_blue ? readParam<int>(nh, "/blue_u_min") : readParam<int>(nh, "/red_s_min"),
                                   init_color_blue ? readParam<int>(nh, "/blue_v_min") : readParam<int>(nh, "/red_v_min"),
                                   init_color_blue ? readParam<int>(nh, "/blue_y_max") : readParam<int>(nh, "/red_h_max"),
                                   init_color_blue ? readParam<int>(nh, "/blue_u_max") : readParam<int>(nh, "/red_s_max"),
                                   init_color_blue ? readParam<int>(nh, "/blue_v_max") : readParam<int>(nh, "/red_v_max"));

    double coeffs[6] = {
        readParam<float>(nh, "/gimbal_trans_x"),
        readParam<float>(nh, "/gimbal_trans_y"),
        readParam<float>(nh, "/gimbal_trans_z"),
        readParam<float>(nh, "/gimbal_offset_pitch"),
        readParam<float>(nh, "/gimbal_offset_yaw"),
        readParam<float>(nh, "/gimbal_k_value")};

    aim_dist::AimDistance d_aim; //设置敌人颜色
    d_aim.init(init_color_blue,
               init_color_blue ? readParam<int>(nh, "/blue_thresh_low") : readParam<int>(nh, "/red_thresh_low"),
               init_color_blue ? readParam<int>(nh, "/blue_channel_g") : readParam<int>(nh, "/red_channel_g"),
               init_color_blue ? readParam<int>(nh, "/blue_filter") : readParam<int>(nh, "/red_filter"));
    double t_start = std::chrono::system_clock::now().time_since_epoch().count();
    do
    {
        if (grab.getMat(frame))
            continue;
#ifdef ORIGINAL_PIC
        cv::imshow("origin", frame);
#endif // ORIGINAL_PIC \
    // writer<<frame;
        double t_deal = std::chrono::system_clock::now().time_since_epoch().count();

        ros::spinOnce();
        if (aros.spin_flag == false)
        {
            aros.updateUseOld();
        }
        CURRENT_STATUS = SMALL_BUFF; //DISTANCE_AIM;

        if (OLD_STATUS != CURRENT_STATUS)
        {
            // LOG_INFO("Status changes from %d to %d\n", OLD_STATUS, CURRENT_STATUS)z;
            switch (OLD_STATUS)
            {
            case AUTO_AIM:
                v_aim.reset();
                break;
            case DISTANCE_AIM:
                d_aim.reset();
                break;
            case SMALL_BUFF:
                wm.reset();
                break;
            case LARGE_BUFF:
                wm.start_t = t_deal;
                wm.reset();
                break;
            default:;
            }
            grab.setCameraParams(CURRENT_STATUS);
        }
        switch (CURRENT_STATUS % 128)
        {
        case AUTO_AIM:
            v_aim.aimAuto(frame, aros.g_msg);
            break; //对于中近距离打击，总是高曝光
        case DISTANCE_AIM:
            d_aim.aimAuto(frame, aros.g_msg);
            break;
        case SMALL_BUFF:
            wm.Hit(frame, aros.g_msg, SMALL_BUFF);
            break;
        case LARGE_BUFF:
            wm.last_t = wm.now_t;
            wm.now_t = t_deal;
            wm.Hit(frame, aros.g_msg, LARGE_BUFF);
            break; //TODO: large_buff(...)
        default:;
        }

        // LOG_GAY_STREAM("========================== After aim =========================");
        grab.setBufferState();          //unlock
        gimbal_pub.publish(aros.g_msg); //云台通信
        aros.clearSpinFlag();
    } while (ros::ok());
    double t_end = std::chrono::system_clock::now().time_since_epoch().count();
    printf("Average running time: %lf ms\n", (t_end - t_start) / 1e6);
    return 0;
}
