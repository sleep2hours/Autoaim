#include "aim_energy/WindMill.hpp"
namespace WINDMILL
{
    void windMill::DirectionJudge()
    {
        if (cnt != 24)
        {
            if (dangle > 0)
                anticlockwize++;
            else
                clockwize++;
        }
        else
        {
            if (clockwize > anticlockwize)
                params.constant_speed*=-1;
            else
                params.constant_speed *=1;
        }
    }
}