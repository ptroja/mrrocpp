/*
 * ecp_st_smooth_move.cpp
 *
 *  Created on: 20-07-2010
 *      Author: kszkudla
 */

#include "ecp_st_scan_move.h"
#include "bcl_t_switcher.h"
#include <string.h>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

/**
 * Subtask constructor
 * @param _ecp_t reference to parent task
 */
ecp_st_scan_move::ecp_st_scan_move(task & _ecp_t):
	ecp_sub_task(_ecp_t),
	bcl_ecp((bcl_t_switcher &)ecp_t){

	std::cout << "ECP_ST_SMOOT_MOVE" << std::endl;

	//Creating new smart pointer to generator object
	bcl_gen = shared_ptr<generator::bclike_gen>(new generator::bclike_gen((bcl_t_switcher &)ecp_t, bcl_ecp.get_vsp_fradia()));
//	bcl_smooth->set_absolute();

}

ecp_st_scan_move::~ecp_st_scan_move() {

}

/**
 * Method called by parent task to start execution of subtask
 */
void ecp_st_scan_move::conditional_execution(){
//	std::cout << "CONDITIONAL EXECUTION" << std::endl;

	std::vector<double>vec;
	vec = msg.stringToRobotPosition(ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string);
//	bcl_smooth->set_debug(true);

	bcl_gen->reset();
	bcl_gen->set_absolute();
	//Loading move destination point
#ifdef JOINT
	bcl_gen->load_absolute_joint_trajectory_pose(vec);
#endif //JOINT

#ifdef EULER
	bcl_gen->load_absolute_euler_zyz_trajectory_pose(vec);
#endif //EULER

	//Starting move
	if(bcl_gen->calculate_interpolate())
		bcl_gen->Move();
	bcl_gen->reset();
}

}

}

}

}

