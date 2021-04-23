#ifndef SENTRY_DECISION_HPP
#define SENTRY_DECISION_HPP
#include <iostream>
#include"../aim_deps/AimDeps.hpp"
#include "../../../serial_com/include/serial_com/comm.h"
#include"../../../serial_com/include/serial_com/detect.h"

/*
enum ROBOT_STATUS{
    AUTO_AIM        = 0,        //近程自瞄
    DISTANCE_AIM    = 1,        //远程自瞄
    LARGE_BUFF      = 2,        //大符打击
    SMALL_BUFF      = 3,        //小符打击
    ANTI_TOP        = 4,        //反陀螺
    SWEEP           = 5         //扫描搜索(未发现敌人)
};*/
class Sentry_decision
{
    public:
    void Sentry_decison();
    int make_decision(int mode,serial_com::comm &msg, serial_com::detect &p_msg );
    private:
    inline bool aim_correctly(float yaw,int radar)//TODO
    {return true;}
    inline bool aim_hero(float yaw,int radar)//TODO
    {
        return true;
    }
    aim_deps::PnP_depended_param *_pnp_params=&aim_deps::pnp_depended_param;
    aim_deps::Sentry_decision _params;
};
#endif