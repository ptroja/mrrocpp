#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_r_conv.h"
#include "ecp/conveyor/ecp_t_test.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp/common/generator/ecp_g_jarosz.h"
#include "ecp/conveyor/ecp_g_test.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

// KONSTRUKTORY
test::test(lib::configurator &_config) : task(_config)
{
	ecp_m_robot = new robot (*this);

	sr_ecp_msg->message("ECP loaded");
}


void test::main_task_algorithm(void)
{
	generator::y_simple ysg(*this, 8);
	ysg.sensor_m = sensor_m;

	for(;;) {
		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("Ruch");
		sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
		ysg.Move();
	}
}

}
} // namespace conveyor

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new conveyor::task::test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


