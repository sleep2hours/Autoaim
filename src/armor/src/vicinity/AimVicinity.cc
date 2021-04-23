#include "vicinity/AimVicinity.hpp"

namespace AimVicinity{

AimVicinity::AimVicinity(bool _blue,int h_min,int s_min,int v_min,int h_max,int s_max,int v_max){
    //如果_blue为true, 则设置敌人为蓝色，否则设置为红色
    //基类中已经构造过了                                       
    _param = aim_deps::Vicinity_param(_blue, h_min, s_min, v_min, h_max, s_max, v_max);
    detecting_module = Detective(_param);
    if(!_blue) detecting_module.setEnemyColor(2);           //设为红色
    else detecting_module.setEnemyColor(0);                 //设为蓝色
    #ifdef _BASE_DEBUGGER
        printf("Aim Vic is initialized.\n");
        if(!_blue) printf("Detective is set to be: enemy is red.\n");
        else printf("Detective is set to be: enemy is blue.\n");
    #endif
}

AimVicinity::~AimVicinity(){
    ;
}

int AimVicinity::aimAuto(cv::Mat &src, serial_com::comm &msg){
    if(msg.status > 127){                                       //status大于127说明敌人是红色
        if(detecting_module._enemy_color == BLUE) detecting_module.setEnemyColor(2);             //当前不是红色，设为红色
    }else{
        if(detecting_module._enemy_color == RED) detecting_module.setEnemyColor(0);             //当前不是蓝色，设为蓝色
    }                                                           //防止一开始设置错误颜色，还有修正的机会
    detecting_module.loadImg(src);
    detecting_module.detect();
    tar_list = detecting_module.get_all_armors();
    // if(tar_list.size()){
    //     _mode = modeDecide(msg.ser);            //判断本次的模式
    //     auto func = std::bind(&AimVicinity::aimPosition, this);
    //     std::thread pnp_thread(func);
    //     aimRecognition(detecting_module.get_gray_img());
    //     pnp_thread.join();                  //两线程合并
    //     switch(_mode){
    //         case aim_base::SEQ_MATCH:       seqMatch(src, msg); break;
    //         case aim_base::FREE_TARGET:     freeTarget(src, msg); break;
    //         default: ;
    //     }
    // }
    // else{
    //     _pit = 0;
    //     _yaw = 0;
    //     msg.status = aim_deps::NO_SHOOTING;
    // }
    if(tar_list.size()){                        //只有识别到了装甲板才会继续执行
        auto func = std::bind(&AimVicinity::aimPosition, this);
        std::thread pnp_thread(func);
        aimRecognition(detecting_module.get_gray_img());
        pnp_thread.join();                  //两线程合并
        if (std::round(msg.z) != 0){
            if (seqMatch(src, msg) == false){
                freeTarget(src, msg);
            }
        }
        else{
            freeTarget(src, msg);
        }
    }
    else{       // 无目标零保护
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
        msg.status = aim_deps::NO_SHOOTING;
    }
    #ifdef DISPLAY_TARGETS
        aimDisplay(src, msg);
        cv::imshow("disp", src);
        char key = cv::waitKey(aim_base::debug_delay);
        if(key == ' ') cv::waitKey(0);
        
        #ifdef _SAVE_DEBUG_VIDEO
            outputVideo<<src;
        #endif  //_SAVE_DEBUG_VIDEO
    #endif  //DISPLAY_TARGTES
    
    optimal = aim_base::NULLPOS;                    //每次循环结束需要重新设置
    return true;                                    //默认高曝光
}
//=======================模式函数===========================//
void AimVicinity::freeTarget(cv::Mat &src, serial_com::comm &msg){
    #ifdef MODE_SPECIFY
        printf("This is free Target Mode.\n");
    #endif
    
    // 决策
    optimal = aim_dec.Armor_decision(tar_list);

    // 弹道
    aimBallistic(msg);
}

//=======================非集成模块函数=========================//
void AimVicinity::aimDisplay(cv::Mat &src, const serial_com::comm msg){
    char str[20];						//绘制所有灯条
    detecting_module.drawArmorPlates(src, tar_list, optimal);			    //绘制装甲板
    //Projection prj;
    snprintf(str, 20, "Delta pitch:%f", msg.y);             //云台转动信息pitch
	cv::putText(src, str,cv::Point(30, 330),
		cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
	snprintf(str, 20, "Delta yaw:%f", msg.x);                 //云台转动信息yaw
	cv::putText(src, str,cv::Point(30, 360),
		cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
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
        snprintf(str, 20, "Delay: %f", now_delay);                     //子弹滞空时间
	    cv::putText(src, str,cv::Point(30, 480),
	    	cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
    }
}    

void AimVicinity::aimRecognition(const cv::Mat src){
    for(size_t i = 0; i<tar_list.size(); ++i){
        classifier.reco(tar_list[i], src);
    }
}
}   //namespace AimVicinity