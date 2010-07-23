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

const double left[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0};
const double right[] = { 0.0, -0.55, -1.37, 0.100, -0.040, 4.627, -1.57, 0.0};
const double start[] = { 0.0, 0.0, -1.37, 0.100, -0.040, 4.627, 0.0, 0.0};


bcl_t_switcher::bcl_t_switcher(lib::configurator &_config):
		task(_config){

//	vsp_fradia = boost::shared_ptr<bcl_fradia_sensor>(new bcl_fradia_sensor(this->config, "[vsp_fradia_sensor]"));
	vsp_fradia = new bcl_fradia_sensor(this->config, "[vsp_fradia_sensor]");
	sensor_m[ecp_mp::sensor::SENSOR_CVFRADIA] = vsp_fradia;
	sensor_m[ecp_mp::sensor::SENSOR_CVFRADIA]->configure_sensor();

	ecp_m_robot = new ecp::irp6ot_m::robot(*this);

	bc_smooth = shared_ptr<generator::bclike_smooth> (new generator::bclike_smooth(*this));
	bc_smooth->set_absolute();

	//dodanie subtaskow do wykonywania
	ecp_sub_task* ecpst;
	ecpst = new ecp_st_smooth_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_SMOOTH_MOVE] = ecpst;

}

bcl_t_switcher::~bcl_t_switcher() {
}

void bcl_t_switcher::mp_2_ecp_next_state_string_handler(void){

	sr_ecp_msg->message("SWITCHER begin");

	if (mp_2_ecp_next_state_string == ecp_mp::task::BCL_MOTION_DIR_STR){
		switch((BCL_MOTION_DIR)mp_command.ecp_next_state.mp_2_ecp_next_state_variant){
			case LEFT:
//				std::vector<double> vec(tmp, tmp + 8);//sizeof(double) * 8);
				vec.assign(left, left + 8);
				bc_smooth->load_absolute_joint_trajectory_pose(vec);
				bc_smooth->calculate_interpolate();
				sr_ecp_msg->message("SWITCHER left");
				break;
			case RIGHT:
//				bc_smooth->load_coordinates(lib::ECP_JOINT, 0.0, -0.55, -1.37, 0.100, -0.040, 4.627, -1.57, 0.0, true);
				vec.assign(right, right + 8);
				bc_smooth->load_absolute_joint_trajectory_pose(vec);
				bc_smooth->calculate_interpolate();
				sr_ecp_msg->message("SWITCHER right");
				break;
			case START:
//				bc_smooth->load_coordinates(lib::ECP_JOINT, 0.0, 0.0, -1.37, 0.100, -0.040, 4.627, 0.0, 0.0, true);
				vec.assign(start, start + 8);
				bc_smooth->load_absolute_joint_trajectory_pose(vec);
				bc_smooth->calculate_interpolate();
				sr_ecp_msg->message("SWITCHER start");
				std::cout << "MESSAGE: " << mp_command.ecp_next_state.mp_2_ecp_next_state_string << std::endl;
				break;
		}
		sr_ecp_msg->message("Move before");
		bc_smooth->Move();
		sr_ecp_msg->message("Move after");
		bc_smooth->reset();
	}

	sr_ecp_msg->message("SWITCHER end");


}

//boost::shared_ptr <bcl_fradia_sensor> bcl_t_switcher::get_vsp_fradia(){
//	return vsp_fradia;
//}

bcl_fradia_sensor*  bcl_t_switcher::get_vsp_fradia(){
	return vsp_fradia;
}

task* return_created_ecp_task(lib::configurator &_config)
{
	return new bcl_t_switcher(_config);
}

}

}

}

}
