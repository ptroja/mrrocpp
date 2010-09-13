// ------------------------------------------------------------------------
//   task/ecp_t_dung.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------


#include <cstdio>
#include <unistd.h>
#include <map>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_t_dung.h"
#include "ecp_g_dung.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

// KONSTRUKTORY
dung::dung(lib::configurator &_config) :
	task(_config) {
	ecp_m_robot = new irp6p_m::robot(*this);
}

void dung::main_task_algorithm(void) {
	generator::dung dg(*this, 4);

	for (;;) {
		sr_ecp_msg->message("NEW SERIES");

		dg.Move();
	}
}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6p_m::task::dung(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

