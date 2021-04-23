#include "Sentrydecision/SentryDecision.hpp"

int Sentry_decision::make_decision(int mode,serial_com::comm &msg,serial_com::detect &p_msg)
{
    p_msg.pitch = msg.y;
    p_msg.yaw = msg.x;
    p_msg.command = 0;
    p_msg.extend = 0;
    if(aim_correctly(msg.x, 0)) // msg.radar))
    {
        // 0 -> msg.blood
        if(0 > _params.blood_limit)
        {
            if(aim_hero(msg.x, 0)) // msg.radar))
            {
                _pnp_params->Hero_score=10;
            }
            else{
                _pnp_params->Distance_multi=2;
            }
            // 0 -> msg.bullet
            if(0 > _params.bullet_limit)
            {

            }
            return mode;
        }
        else{
            _pnp_params->Distance_multi=2;
            p_msg.command=100;
            //std::cout<<p_msg.command<<std::endl;
            return mode;
        }
    }
    else{
        return 5;
    }
}