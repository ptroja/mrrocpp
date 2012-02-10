/**
 * \file bcl_t_switcher.cc
 * \brief Main ECP task methods definition
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "bcl_t_switcher.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_st_scan_move.h"
#include "ecp_st_position_move.h"

#include "ecp_mp_st_scan_move.h"
#include "ecp_mp_st_position_move.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

bcl_t_switcher::bcl_t_switcher(lib::configurator &_config):
	common::task::task(_config)
{
	std::cout << "TWORZE BCL SWITCHER" << std::endl;

	//Creating new fradia sensor
	vsp_fradia = new bcl_fradia_sensor(this->config, "[vsp_fradia_sensor]");
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA]->configure_sensor();


#ifdef IRP6_OT
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
#endif//IRP6_OT

#ifdef IRP6_P
	ecp_m_robot = (boost::shared_ptr<robot_t) new ecp::irp6p_m::robot(*this);
#endif//IRP6_P

	//Adding additional subtasks
	subtask::subtask* ecpst;
	ecpst = new subtask::ecp_st_scan_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_SCAN_MOVE] = ecpst;

	ecpst = new subtask::ecp_st_position_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_POSITION_MOVE] = ecpst;

	std::cout << "KONIEC KONSTRUKTORA BCL SWITCHER" << std::endl;
}

bcl_t_switcher::~bcl_t_switcher() {
	std::cout << "ZABIJAM BCL SWITCHER" << std::endl;
}

void bcl_t_switcher::mp_2_ecp_next_state_string_handler(void){
}

bcl_fradia_sensor*  bcl_t_switcher::get_vsp_fradia(){
	return vsp_fradia;
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new bcl_t_switcher(_config);
}

}

}

}

}
