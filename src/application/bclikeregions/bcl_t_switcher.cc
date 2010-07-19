/*
 * bcl_t_switcher.cc
 *
 *  Created on: Jul 6, 2010
 *      Author: kszkudla
 */

#include "bcl_t_switcher.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

bcl_t_switcher::bcl_t_switcher(lib::configurator &_config):
		task(_config){

	ecp_m_robot = new ecp::irp6ot_m::robot(*this);

	bc_smooth = shared_ptr<generator::bclike_smooth> (new generator::bclike_smooth(*this, true));
//	bc_smooth = shared_ptr<generator::smooth> (new generator::smooth(*this, true));
	bc_smooth->set_absolute();
}

bcl_t_switcher::~bcl_t_switcher() {
}

void bcl_t_switcher::mp_2_ecp_next_state_string_handler(void){

	sr_ecp_msg->message("SWITCHER begin");

	if (mp_2_ecp_next_state_string == ecp_mp::task::BCL_MOTION_DIR_STR){
		switch((BCL_MOTION_DIR)mp_command.ecp_next_state.mp_2_ecp_next_state_variant){
			case LEFT:
				bc_smooth->load_coordinates(lib::ECP_JOINT, 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0, true);
				sr_ecp_msg->message("SWITCHER left");
				break;
			case RIGHT:
				bc_smooth->load_coordinates(lib::ECP_JOINT, 0.0, -0.55, -1.37, 0.100, -0.040, 4.627, -1.57, 0.0, true);
				sr_ecp_msg->message("SWITCHER right");
				break;
			case START:
				bc_smooth->load_coordinates(lib::ECP_JOINT, 0.0, 0.0, -1.37, 0.100, -0.040, 4.627, 0.0, 0.0, true);
				sr_ecp_msg->message("SWITCHER start");
				break;
		}
		sr_ecp_msg->message("Move before");
		bc_smooth->Move();
		sr_ecp_msg->message("Move after");
		bc_smooth->reset();
	}

	sr_ecp_msg->message("SWITCHER end");


}

task* return_created_ecp_task(lib::configurator &_config)
{
	return new bcl_t_switcher(_config);
}

}

}

}

}
