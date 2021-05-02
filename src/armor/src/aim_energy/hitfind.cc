#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    bool windMill::FindHitPoint(cv::Mat &img)
    {
        if (GetContours(img))
        {
            hit_point = armors[0].center;
            now_angle = CalAngle(hit_point, center);
            dangle_deg = (now_angle - last_angle) / ((now_t - last_t));
            dangle_rad = abs(dangle_deg * Pi / 180);
            UndisortPoint(center);
            
            if (dangle_deg < 2.09 && dangle_deg > 0.52 && mode == big_buff)
            {
                std::cout << dangle_deg << std::endl;
                if (anglevelocity_rad.size() > 30)
                {
                    anglevelocity_rad.pop_front();
                    t_list.pop_front();
                }
                anglevelocity_rad.push_back(dangle_rad);
                t_list.push_back(now_t);
            }
            last_angle = now_angle;
        }
        else
            return false;
    }

}
