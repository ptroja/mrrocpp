// ------------------------------------------------------------------------
//   task/ecp_t_dung.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_postument/task/ecp_t_dung.h"
#include "ecp/irp6_postument/generator/ecp_g_dung.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

// KONSTRUKTORY
dung::dung(lib::configurator &_config) : task(_config)
{
	ecp_m_robot = new robot (*this);
}

void dung::main_task_algorithm(void)
{
	generator::dung dg(*this, 4);

	for(;;) {
		sr_ecp_msg->message("NEW SERIES");

		dg.Move();
	}
}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6p::task::dung(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

