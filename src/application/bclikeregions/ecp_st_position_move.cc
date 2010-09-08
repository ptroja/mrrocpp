/*
 * ecp_st_smooth_move.cpp
 *
 *  Created on: 20-07-2010
 *      Author: kszkudla
 */

#include "ecp_st_position_move.h"
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
ecp_st_position_move::ecp_st_position_move(task & _ecp_t):
	ecp_sub_task(_ecp_t),
	bcl_ecp((bcl_t_switcher &)ecp_t){

	std::cout << "ECP_ST_POSITION_MOVE" << std::endl;

	//Creating new smart pointer to generator object
//	smooth = shared_ptr<generator::newsmooth>(new generator::bclike_smooth((bcl_t_switcher &)ecp_t, bcl_ecp.get_vsp_fradia()));

#ifdef IRP6_OT
#ifdef JOINT
	smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(ecp_t, lib::ECP_JOINT, 7));
#endif//JOINT

#ifdef EULER
	smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6));
#endif//EULER

#endif//IRP6_OT

#ifdef IRP6_P
#ifdef JOINT
	smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(ecp_t, lib::ECP_JOINT, 6));
#endif//JOINT

#ifdef EULER
	smooth = shared_ptr<generator::newsmooth> (new generator::newsmooth(ecp_t, lib::ECP_XYZ_EULER_ZYZ, 6));
#endif//EULER

#endif//IRP6_P


}

ecp_st_position_move::~ecp_st_position_move() {

}

/**
 * Method called by parent task to start execution of subtask
 */
void ecp_st_position_move::conditional_execution(){
//	std::cout << "CONDITIONAL EXECUTION" << std::endl;

	std::vector<double>vec;
	vec = msg.stringToRobotPosition(ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string);

	smooth->reset();
	smooth->set_absolute();

	//Loading move destination point
#ifdef JOINT
	smooth->load_absolute_joint_trajectory_pose(vec);
#endif //JOINT

#ifdef EULER
	smooth->load_absolute_euler_zyz_trajectory_pose(vec);
#endif //EULER

	//Starting move
	if(smooth->calculate_interpolate())
		smooth->Move();
	smooth->reset();
}

}

}

}

}

