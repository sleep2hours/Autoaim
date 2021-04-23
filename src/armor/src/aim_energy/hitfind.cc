#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    bool windMill::FindHitPoint(cv::Mat &img)
    {
        if (GetContours(img))
        {
            hit_point = armors[0].center;
            now_angle = CalAngle(hit_point, center);
            dangle_deg = (now_angle - last_angle) / ((now_t - last_t) / 1e3);
            dangle_rad = abs(dangle_deg * Pi / 180);
            if (dangle_rad < 2.09 && dangle_rad > 0.52 && mode == big_buff)
            {
                if (anglevelocity_rad.size() > 10)
                {
                    anglevelocity_rad.pop_front();
                    t_list.pop_front();
                }
                anglevelocity_rad.push_back(dangle_rad);
                t_list.push_back(now_t - start_t);
            }
            last_angle = now_angle;
        }
        else
            return false;
    }

}
