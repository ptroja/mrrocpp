#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/task/ecp_t_box_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
box::box(lib::configurator &_config) : common::task::task(_config),
	smoothgen(*this, true)
{
	ecp_m_robot = new robot(*this);
}

void box::main_task_algorithm(void )
{
	smoothgen.set_absolute();
	smoothgen.load_file_with_path("/mnt/mrroc/trj/box_euler.trj");
	smoothgen.Move();
	smoothgen.reset();

	ecp_termination_notice();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::box(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

