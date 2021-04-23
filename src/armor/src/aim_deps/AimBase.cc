#include "aim_deps/AimBase.hpp"

namespace aim_base{
AimBase::AimBase(){
    optimal         = NULLPOS;
    _is_enemy_blue  = true;                                         //默认敌人是蓝色
    now_delay       = 0.0;
    last_match      = 0;

    #ifdef BASE_CNT_TIME
        sum_t = 0.0;
        _cnt = 0.0;
    #endif //BASE_CNT_TIME
}

AimBase::~AimBase(){
    ;
}

//seqMatch是默认不发弹的
/// @todo 电控发什么
bool AimBase::seqMatch(cv::Mat &src, serial_com::comm &msg){
    int ideal_num = std::round(msg.z);          // msg.z 为电控所发的需要追踪序列号
    bool result = false;                        // 默认找不到对应的车号，返回false
    for(aim_deps::Armor &arm: tar_list){
        if(arm.armor_number == ideal_num){
            result = true;
        }
    }
    if(result == false){
        return false;
    }

    // 记录本次操作手指定车号为ideal_num，并只对ideal_num的所有车进行查找
    last_match = ideal_num;
    optimal = aim_dec.Armor_decision(tar_list, ideal_num);

    // 如果决策无法确定车号为ideal_num 的决策结果，则对其他车号进行搜索
    if(optimal == NULLPOS){                     // track_num对应的装甲板没有被找到
        optimal = aim_dec.Armor_decision(tar_list, 0, ideal_num);
    }
    aimBallistic(msg);
    return true;
}

void AimBase::aimPosition(){
    pos_getter.batchProcess(tar_list);
}

void AimBase::aimBallistic(serial_com::comm &msg){
    // printf("Aim Base x, y, z: %f, %f, %f\n", msg.x, msg.y, msg.z);
    if(tar_list.size() == 0){       // 视野内没有目标，则退出
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
        msg.status = aim_deps::NO_SHOOTING;
        return;
    }
    if(optimal != NULLPOS){                   //存在一个最佳决策
        /// TODO:
        cv::Point3f pos = tar_list[optimal].t_vec;

        /// 计算： 是否进行静止装甲板滤波
        pos_getter.calcBallistic(pos, msg);
        _x = pos.x;
        _y = pos.y;
        _z = pos.z;
        msg.x = _x;
        msg.y = _y;
        msg.z = _z;
    }
    else{                           // 无最佳决策
        //printf("Optimal pos is null[%d], can nnot fire.\n", optimal);
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
        msg.status = aim_deps::NO_SHOOTING;
    }
    // printf("After ballistics x, y, z: %f, %f, %f\n", msg.x, msg.y, msg.z);
}

/////////////////////////////////////////////////////////////
//=========================哨兵=============================//
/////////////////////////////////////////////////////////////
#ifdef SENTRYDECISION
int AimBase::Switch_mode()
{  
    if(Last_10_targets.size() !=0&&Last_10_targets.back().armor_number==tar_list[optimal].armor_number)
    {
        Last_10_targets.emplace_back(tar_list[optimal]);
        std::vector<float> distance_judge;
        std::vector<float> angle_judge_1;
        std::vector<float>angle_judge_2;
        for(auto &target : Last_10_targets)
        {
            distance_judge.emplace_back(target.t_vec.z);
            angle_judge_1.emplace_back(target.r_vec.at<float>(0,0));
            angle_judge_2.emplace_back(target.r_vec.at<float>(1,0));
        }
        double sum = std::accumulate(std::begin(distance_judge), std::end(distance_judge), 0.0);
    	double mean =  sum / distance_judge.size(); //均值
    	double accum  = 0.0;
    	std::for_each (std::begin(distance_judge), std::end(distance_judge), [&](const double d) {
    		accum  += (d-mean)*(d-mean);
    	});
    	double distance_stdev = sqrt(accum/(distance_judge.size()-1)); //方差

        sum = std::accumulate(std::begin(angle_judge_1), std::end(angle_judge_1), 0.0);
    	 mean =  sum / angle_judge_1.size(); //均值
    	 accum  = 0.0;
    	std::for_each (std::begin(angle_judge_1), std::end(angle_judge_1), [&](const double d) {
    		accum  += (d-mean)*(d-mean);
    	});
    	double angle1_stdev = sqrt(accum/(angle_judge_2.size()-1)); //方差
        sum = std::accumulate(std::begin(angle_judge_2), std::end(angle_judge_2), 0.0);
    	 mean =  sum / angle_judge_2.size(); //均值
    	 accum  = 0.0;
    	std::for_each (std::begin(angle_judge_2), std::end(angle_judge_2), [&](const double d) {
    		accum  += (d-mean)*(d-mean);
    	});
    	double angle2_stdev = sqrt(accum/(angle_judge_1.size()-1)); //方差
        //std::cout<<angle1_stdev+angle2_stdev<<std::endl;
        if(Last_10_targets.size()>30){Last_10_targets.erase(Last_10_targets.begin());}
        if(distance_stdev<100&&angle1_stdev+angle2_stdev>5)
        {
            return 3;
        }
        else{
            return tar_list[optimal].t_vec.z>5000?1:0;
        }
    }
    else{
        std::vector<aim_deps::Armor>().swap(Last_10_targets);
        Last_10_targets.emplace_back(tar_list[optimal]);
        return tar_list[optimal].t_vec.z>5000?1:0;
    }
}
#endif  //SENTRYDECISION

}       //namespace aim_base
