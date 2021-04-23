#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    void windMill::clear()
    {
        fin_contours.clear();
        R_contours.clear();
        armors.clear();
        fins.clear();
        center_pt.clear();
    }

    void windMill::reset()
    {
        anglevelocity_rad.clear();
        t_list.clear();
    }
}
