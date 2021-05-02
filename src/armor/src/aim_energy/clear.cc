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
        iterator=0;
        last_loss=0.0;
        anglevelocity_rad.clear();
        t_list.clear();
    }
}
