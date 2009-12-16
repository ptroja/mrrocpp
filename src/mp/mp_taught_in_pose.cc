// -------------------------------------------------------------------------
//                              mp.cc
//
// MP Master Process - methods
//
// -------------------------------------------------------------------------
// Funkcje do konstruowania procesow MP

#include <unistd.h>
#include <string.h>

#include <boost/foreach.hpp>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp_taught_in_pose.h"

#include "ecp_mp/task/ecp_mp_t_player.h"

using namespace std;

namespace mrrocpp {
namespace mp {
namespace common {




mp_taught_in_pose::mp_taught_in_pose(void)
{}

mp_taught_in_pose::mp_taught_in_pose(lib::POSE_SPECIFICATION at, double mt, double* c) :
        arm_type(at), motion_time(mt)
{
    memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}

mp_taught_in_pose::mp_taught_in_pose(lib::POSE_SPECIFICATION at, double mt,
                                     double* c, double* irp6p_c) :
        arm_type(at), motion_time(mt)
{
    memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
    memcpy(irp6p_coordinates, irp6p_c, MAX_SERVOS_NR*sizeof(double));
}

mp_taught_in_pose::mp_taught_in_pose(lib::POSE_SPECIFICATION at, double mt,
                                     int e_info, double* c) :
        arm_type(at), motion_time(mt), extra_info(e_info)
{ // by Y
    memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ##############################################################
// ##############################################################
//                              CIALA METOD dla generatorow
// ##############################################################
// ##############################################################
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

} // namespace common
} // namespace mp
} // namespace mrrocpp
