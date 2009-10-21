// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__MP_H)
#define __MP_H

#include <map>
#include <stdio.h>

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/configurator.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class robot;
}

namespace common {

typedef std::pair<const lib::robot_name_t, robot::robot*> robot_pair_t;
typedef std::map <lib::robot_name_t, robot::robot*> robots_t;

// ---------------------------------------------------------------
class MP_main_error
{ // Klasa obslugi bledow poziomie MP
	public:
		const lib::ERROR_CLASS error_class;
		const uint64_t mp_error;
		MP_main_error(lib::ERROR_CLASS err0, uint64_t err1, const char *file, int line) :
			error_class(err0), mp_error(err1)
		{
			fprintf(stderr, "ECP_MP_main_error @ %s:%d\n", file, line);
		}
#define MP_main_error(e0,e1)	MP_main_error((e0),(e1), __FILE__, __LINE__)
};
// ---------------------------------------------------------------

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

// to fix forward declaration issues
#include "mp/mp_generator.h"
#include "mp/mp_task.h"
#include "mp/mp_robot.h"
#include "lib/com_buf.h"

#endif
