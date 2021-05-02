#include "distance/AimDistance.hpp"

namespace aim_dist{

AimDistance::AimDistance(){
    #ifdef _BASE_DEBUGGER
        printf("AimDistance is initialized.\n");
        if(!_blue) printf("Match is set to be: enemy is red.\n");
        else printf("Match is set to be: enemy is blue.\n");
    #endif // BASE_DEBUGGER
    _is_reset = false;
    #ifdef DISPLAY_TARGETS
        _delay = 1;//aim_base::debug_delay;       //按下e键之后，会以_delay ms每帧的方式进行播放(并修改slow_judge)
        slow_judge = false;        //为true时，会以600ms每帧的方式播放
    #endif // DISPLAY_TARGETS
}

AimDistance::~AimDistance(){
    ;
}

void AimDistance::init(bool _blue, int thresh_low, int ch_diff, int filter)
{
    if(!_blue) match.setEnemyColor(false, thresh_low, ch_diff, filter);               //敌人颜色设为红色
    else match.setEnemyColor(true, thresh_low, ch_diff, filter);                 //敌人颜色设为蓝色
}


//==========================主要的逻辑框架=========================//
//========================远距离打击自瞄集成========================//
    
void AimDistance::aimAuto(cv::Mat &src, serial_com::comm &msg){
    serial_com::comm msg_bk = msg;
    match.saveImg(src);
    match.findPossible();
    amp.matchAll(match.matches, match.possibles, tar_list);//查找匹配灯条
    aimDebugDisplay(src, msg);
    if(tar_list.size()){                        //只有识别到了装甲板才会继续执行
        aimPosition();
        cv::Mat gray_img;
        cv::cvtColor(src,gray_img,cv::COLOR_BGR2GRAY);
        aimRecognition(gray_img);
        freeTarget(src, msg);
        if (_is_reset == true){
            _is_reset = false;
        }
       
    }
    else{       // 无目标零保护
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
        msg.status = aim_deps::NO_SHOOTING;
        if (_is_reset == false){        // 防止无目标多次reset
            _is_reset = true;
        }
    }
    
    #ifdef DISPLAY_TARGETS
        aimDisplay(src, msg_bk);
        cv::Mat show = src.clone();
        cv::imshow("disp", show);
        key = cv::waitKey(_delay);
        if(key ==' ') {
            cv::waitKey(0);
        }
        else if(key == 'e') {
            slow_judge = !slow_judge;
            if(slow_judge) _delay = 700;
            else _delay = aim_base::debug_delay;
        }
    #endif  //DISPLAY_TARGETS
    optimal = aim_base::NULLPOS;                    // 最佳装甲板不保存其值
}

//=======================模式函数===========================//
void AimDistance::freeTarget(cv::Mat &src, serial_com::comm &msg){
    #ifdef MODE_SPECIFY
        printf("This is free Target Mode.\n");
    #endif
    optimal = aim_dec.Armor_decision(tar_list);
    aim_deps::Armor& arm = tar_list[optimal];
    camera_p = Eigen::Vector3d(arm.t_vec.x, arm.t_vec.y, arm.t_vec.z);
    // ===================== 需要重新测一下预测 =======================
    // if (false) {
    //     if (aimp.translatePredict(arm.t_vec, msg, camera_p) == true) {
    //         arm.t_vec.x = camera_p(0);
    //         arm.t_vec.y = camera_p(1);
    //         arm.t_vec.z = camera_p(2);
    //     }
    // }
    aimBallistic(msg);
}

void AimDistance::aimDebugDisplay(cv::Mat &src, const serial_com::comm & msg){
    char str[20];
	snprintf(str, 20, "Delta yaw:%f", msg.x);                 //云台转动信息yaw
	cv::putText(src, str,cv::Point(30, 360),
		cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
    snprintf(str, 20, "Delta pitch:%f", msg.y);             //云台转动信息pitch
	cv::putText(src, str,cv::Point(30, 330),
		cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
    snprintf(str, 20, "Delay: %f", msg.z);                     //子弹滞空时间
	    cv::putText(src, str,cv::Point(30, 480),
	    	cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
}

//=======================非集成模块函数=========================//
void AimDistance::aimDisplay(cv::Mat &src, const serial_com::comm &msg){
    char str[20];
    amp.drawArmorPlates(src, tar_list, optimal);			    //绘制装甲板
    if(optimal >= 0){
        snprintf(str, 20, "X: %f", tar_list[optimal].t_vec.x);      //最佳装甲板位置x
	    cv::putText(src, str,cv::Point(30, 390),
	    	cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
	    snprintf(str, 20, "Y: %f", tar_list[optimal].t_vec.y);      //最佳装甲板位置y
	    cv::putText(src, str,cv::Point(30, 420),
	    	cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
	    snprintf(str, 20, "Z: %f", tar_list[optimal].t_vec.z);      //最佳装甲板位置z
	    cv::putText(src, str,cv::Point(30, 450),
	    	cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
    }
    #ifdef DRAW_CONTOUR
        match.drawContour(src); 
    #endif
    match.drawLights(src);							//绘制所有灯条
    // Eigen::Quaterniond c2w = angle2Quat(msg.y, - msg.x, 0.0);
    // Eigen::Vector3d pw(0, 0, 10);
    // Eigen::Vector3d cam = c2w.conjugate() * pw * 1000;
    Eigen::Matrix3d this_K;
    this_K << 
        1776.67168581218, 0, 720,
        0, 1778.59375346543, 540,
        0, 0, 1;
    Eigen::Vector3d cam = camera_p / camera_p(2);
    Eigen::Vector3d proj = this_K * cam;
    cv::circle(src, cv::Point(proj(0), proj(1)), 5, cv::Scalar(0, 255, 0), -1);
}    

void AimDistance::aimRecognition(const cv::Mat &src){
    for(size_t i = 0; i<tar_list.size(); ++i){
        classifier.reco(tar_list[i], src);
    }
}
}//namespace aim_dist
