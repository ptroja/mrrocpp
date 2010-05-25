#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_smooth_test.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

//Constructors
smooth_test::smooth_test(lib::configurator &_config) :
	task(_config) {
	ecp_m_robot = new irp6ot_m::robot(*this);

	//delay(20000);
	smoothgen2 = new common::generator::smooth(*this, true);
	sr_ecp_msg->message("ECP loaded smooth_test");
}
;

void smooth_test::main_task_algorithm(void) {

	sr_ecp_msg->message("ECP smooth_test ready");

	//smoothgen2->set_relative();
	smoothgen2->set_absolute();
	smoothgen2->load_file_with_path(
			"/net/olin/mnt/mrroc/src/application/smooth-tester/trj/smooth2test2.trj");
	smoothgen2->Move();
	smoothgen2->reset();

	ecp_termination_notice();
}
;

} // namespace task
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::smooth_test(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


