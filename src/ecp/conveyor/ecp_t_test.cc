#include <stdio.h>

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_t_test.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/conveyor/ecp_g_test.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

// KONSTRUKTORY
ecp_task_conveyor_test::ecp_task_conveyor_test(configurator &_config) : ecp_task(_config)
{}

ecp_task_conveyor_test::~ecp_task_conveyor_test()
{}

// methods for ECP template to redefine in concrete classes
void ecp_task_conveyor_test::task_initialization(void)
{
	ecp_m_robot = new ecp_conveyor_robot (*this);

	sr_ecp_msg->message("ECP loaded");
}


void ecp_task_conveyor_test::main_task_algorithm(void)
{
	y_simple_generator ysg(*this, 8);
	ysg.sensor_m = sensor_m;

	for(;;) {
		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("Ruch");
		sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
		ysg.Move();
	}
}
} // namespace conveyor

namespace common {
namespace task {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new conveyor::ecp_task_conveyor_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


