/**
 * \file ecp_st_position_move.cc
 * \brief Position move subtask method definition
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "ecp_st_position_move.h"
#include "bcl_t_switcher.h"
#include <string.h>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace sub_task {


ecp_st_position_move::ecp_st_position_move(task::task & _ecp_t):
	sub_task(_ecp_t),
	bcl_ecp((task::bcl_t_switcher &)ecp_t){

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


void ecp_st_position_move::conditional_execution(){

	std::vector<double>vec;
	vec = msg.stringToRobotPosition(ecp_t.mp_command.ecp_next_state.data);

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

