/**
 * \file ecp_st_scan_move.cc
 * \brief Scanning move subtask class methods definition
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "ecp_st_scan_move.h"
#include "bcl_t_switcher.h"
#include <string.h>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace subtask {


ecp_st_scan_move::ecp_st_scan_move(task::task & _ecp_t):
	subtask(_ecp_t),
	bcl_ecp((task::bcl_t_switcher &)ecp_t){

	std::cout << "ECP_ST_SMOOT_MOVE" << std::endl;

	//Creating new smart pointer to generator object
	bcl_gen = shared_ptr<generator::bclike_gen>(new generator::bclike_gen((task::bcl_t_switcher &)ecp_t, bcl_ecp.get_vsp_fradia()));

}

ecp_st_scan_move::~ecp_st_scan_move() {

}


void ecp_st_scan_move::conditional_execution(){

	std::vector<double>vec;
	vec = msg.stringToRobotPosition(ecp_t.mp_command.ecp_next_state.sg_buf.data);

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

