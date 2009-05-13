#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_box_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
box::box(lib::configurator &_config): common::task::task(_config){
  smoothgen = NULL;
};
//Desctructor
box::~box(){

};

//methods for ECP template to redefine in concrete classes
void box::task_initialization(void) {

	ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	smoothgen = new common::generator::smooth(*this, true);
	sr_ecp_msg->message("ECP loaded box");
};

void box::main_task_algorithm(void ){
	sr_ecp_msg->message("ECP box ready");
	//ecp_wait_for_start();

	smoothgen->set_absolute();
	if (smoothgen->load_file_with_path("/mnt/mrroc/trj/box_euler.trj")) {
	  smoothgen->Move();
	}
	smoothgen->reset();

	ecp_termination_notice();
	//ecp_wait_for_stop();
};

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

