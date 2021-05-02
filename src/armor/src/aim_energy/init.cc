#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    windMill::windMill(bool blue)
    {
        iterator = 0;
        last_loss = 0.0;
        cnt = 0;
        _isblue = blue;
        cnt = clockwize = anticlockwize = 0;
        clear();
        anglevelocity_rad.clear();
        t_list.clear();
    }

    windMill::~windMill() { ; }
}
