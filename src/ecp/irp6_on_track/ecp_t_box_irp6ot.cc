#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_box_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

//Constructors
ecp_t_box_irp6ot::ecp_t_box_irp6ot(configurator &_config): common::task::ecp_task(_config){
  smoothgen = NULL;
};
//Desctructor
ecp_t_box_irp6ot::~ecp_t_box_irp6ot(){

};

//methods for ECP template to redefine in concrete classes
void ecp_t_box_irp6ot::task_initialization(void) {

	ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	smoothgen = new common::generator::ecp_smooth_generator(*this, true);
	sr_ecp_msg->message("ECP loaded box");
};

void ecp_t_box_irp6ot::main_task_algorithm(void ){
	sr_ecp_msg->message("ECP box ready");
	//ecp_wait_for_start();

	smoothgen->set_absolute();
	if (smoothgen->load_file_with_path("/mnt/mrroc/MRROC++/trj/box_euler.trj")) {
	  smoothgen->Move();
	}
	smoothgen->reset();

	ecp_termination_notice();
	//ecp_wait_for_stop();
};

} // namespace irp6ot

namespace common {
namespace task {

ecp_task* return_created_ecp_task(configurator &_config){
	return new irp6ot::ecp_t_box_irp6ot(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

