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

/**
 * Class constructor, creating FraDIA sensor, smooth generator, insance
 * of robot object. Also take care about creating subtasks
 * @param _config reference to configuration file parser object
 */

bcl_t_switcher::bcl_t_switcher(lib::configurator &_config):
		task(_config){
	std::cout << "TWORZE BCL SWITCHER" << std::endl;

	//Creating new fradia sensor
	vsp_fradia = new bcl_fradia_sensor(this->config, "[vsp_fradia_sensor]");
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA]->configure_sensor();

#ifdef IRP6_OT
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);

#ifdef JOINT
	bc_smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(*this, lib::ECP_JOINT, 7));
#endif//JOINT

#ifdef EULER
	bc_smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(*this, lib::ECP_XYZ_EULER_ZYZ, 6));
#endif//EULER

#endif//IRP6_OT

#ifdef IRP6_P
	ecp_m_robot = new ecp::irp6p_m::robot(*this);

#ifdef JOINT
	bc_smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(*this, lib::ECP_JOINT, 6));
#endif//JOINT

#ifdef EULER
	bc_smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(*this, lib::ECP_XYZ_EULER_ZYZ, 6));
#endif//EULER

#endif//IRP6_P

	bc_smooth->set_absolute();

	//Adding additional subtasks
	ecp_sub_task* ecpst;
	ecpst = new ecp_st_smooth_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_SMOOTH_MOVE] = ecpst;

}

/**
 * Class destuctor, do the celaning up
 */
bcl_t_switcher::~bcl_t_switcher() {
	std::cout << "ZABIJAM BCL SWITCHER" << std::endl;
	delete ecp_m_robot;
}

/**
 * Method for handling communication with MP
 */
void bcl_t_switcher::mp_2_ecp_next_state_string_handler(void){

	sr_ecp_msg->message("SWITCHER begin");


	std::cout << "SWITCHER begin" << std::endl;

	//TODO: Jakos to przemielic bo i tak wykorzystywane jest tylko START
	if (mp_2_ecp_next_state_string == ecp_mp::task::BCL_MOTION_DIR_STR){

//		vec.assign(left, left + VEC_SIZE);
		vec = msg.stringToRobotPosition(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
		std::cout << "ODCZYTANE " << vec.size() << std::endl;
		for(std::vector<double>::iterator it = vec.begin(); it != vec.end(); ++it)
			std::cout << *it << " ";
		std::cout << std::endl;

		#ifdef JOINT
			bc_smooth->load_absolute_joint_trajectory_pose(vec);
//			bc_smooth->set_debug(true);
		#endif//JOINT

		#ifdef EULER
			std::cout << "load trajectory" << std::endl;
			bc_smooth->load_absolute_euler_zyz_trajectory_pose(vec);
		#endif//EULER

		std::cout << "Interpolate" << std::endl;
		if(bc_smooth->calculate_interpolate()){
			sr_ecp_msg->message("Move before");
			bc_smooth->Move();
			sr_ecp_msg->message("Move after");
		}
		bc_smooth->reset();
	}

	sr_ecp_msg->message("SWITCHER end");

}

/**
 * Method used by subtask to get access to FraDIA sensor
 * @return pointer to fradia sensor structure
 */
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
