// -------------------------------------------------------------------------
//                            mp.h
// Definicje struktur danych i metod dla procesow MP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#if !defined(__MP_H)
#define __MP_H

#if defined(__QNXNTO__)
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#endif


#include "ecp/common/ecp_robot.h"
#include "ecp_mp/ecp_mp_task.h"
#include "lib/configurator.h"
#include "lib/timer.h"

namespace mrrocpp {
namespace mp {
namespace common {


//class base;
class robot;

enum WAIT_FOR_STOP_ENUM {
	MP_EXIT,
	MP_THROW
};

// struktura zwracana przez funkcje mp_receive_ecp_pulse
typedef struct mp_receive_ecp_pulse_return {
	uint32_t nd; // deskryptor wezla na ktorym jest powolane ECP
	pid_t ECP_pid;
	bool rt;
	int32_t scoid; // server connection id
	char pulse_code;
	int rcvid;
	uint64_t e; // errno
} mp_receive_ecp_pulse_return_t;

// struktura wykorzystywana przez funkcje mp_receive_pulse
typedef struct mp_receive_pulse_struct {
	_msg_info msg_info;
	_pulse_msg pulse_msg;
	int rcvid;
	uint64_t e;       // Kod bledu systemowego
} mp_receive_pulse_struct_t;

// ---------------------------------------------------------------
class MP_main_error
{ // Klasa obslugi bledow poziomie MP
	public:
		const ERROR_CLASS error_class;
		const uint64_t mp_error;
		MP_main_error(ERROR_CLASS err0, uint64_t err1, const char *file, int line) :
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
		POSE_SPECIFICATION arm_type;
		double motion_time;
		double coordinates[MAX_SERVOS_NR];
		double irp6p_coordinates[MAX_SERVOS_NR];

		int extra_info; // by Y uzupelnienie struktury o dodatkowe pole, do dowolnego wykorzystania

		mp_taught_in_pose (void);
		mp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c);

		mp_taught_in_pose (POSE_SPECIFICATION at, double mt, double* c, double* irp6p_c);

		mp_taught_in_pose (POSE_SPECIFICATION at, double mt, int e_info, double* c);
};
// ------------------------------------------------------------------------

} // namespace common
} // namespace mp
} // namespace mrrocpp



// to fix forward declaration issues
#include "mp/mp_generator.h"
#include "mp/mp_task.h"
#include "mp/mp_robot.h"
#include "../lib/com_buf.h"

#endif
