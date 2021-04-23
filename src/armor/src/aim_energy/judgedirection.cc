#include "aim_energy/WindMill.hpp"
namespace WINDMILL
{
    void windMill::DirectionJudge()
    {
        if (cnt != 24)
        {
            if (dangle_deg > 0)
                anticlockwize++;
            else
                clockwize++;
        }
        else
        {
            if (clockwize > anticlockwize)
                direction = true;
            else
                direction = false;
        }
    }
}