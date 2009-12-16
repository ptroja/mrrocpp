// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__MP_TAUGHT_IN_POSE_H)
#define __MP_TAUGHT_IN_POSE_H

#include <map>
#include <stdio.h>

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/configurator.h"
#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class robot;
}

namespace common {


// na podstawie ecp_taught_in_pose
// ------------------------------------------------------------------------
class mp_taught_in_pose {
	public:
		lib::POSE_SPECIFICATION arm_type;
		double motion_time;
		double coordinates[MAX_SERVOS_NR]; // irp6ot coordinates
		double irp6p_coordinates[MAX_SERVOS_NR]; // irp6p coordinates

		int extra_info; // by Y uzupelnienie struktury o dodatkowe pole, do dowolnego wykorzystania

		mp_taught_in_pose (void);
		mp_taught_in_pose (lib::POSE_SPECIFICATION at, double mt, double* c);

		mp_taught_in_pose (lib::POSE_SPECIFICATION at, double mt, double* c, double* irp6p_c);

		mp_taught_in_pose (lib::POSE_SPECIFICATION at, double mt, int e_info, double* c);
};
// ------------------------------------------------------------------------

} // namespace common

} // namespace mp
} // namespace mrrocpp


#endif
