#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    windMill::windMill(bool blue)
    {
        cnt=0;
        _isblue = blue;
        cnt = clockwize = anticlockwize = 0;
        clear();
        anglevelocity_rad.clear();
        t_list.clear();
    }

    windMill::~windMill() { ; }
}
