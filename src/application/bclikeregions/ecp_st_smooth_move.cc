/*
 * ecp_st_smooth_move.cpp
 *
 *  Created on: 20-07-2010
 *      Author: kszkudla
 */

#include "ecp_st_smooth_move.h"
#include "bcl_t_switcher.h"


namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_st_smooth_move::ecp_st_smooth_move(task & _ecp_t):
	ecp_sub_task(_ecp_t),
	bcl_ecp((bcl_t_switcher &)ecp_t){

	bcl_smooth = shared_ptr<generator::bclike_smooth>(new generator::bclike_smooth((bcl_t_switcher &)ecp_t, bcl_ecp.get_vsp_fradia()));

}

ecp_st_smooth_move::~ecp_st_smooth_move() {
	// TODO Auto-generated destructor stub
}

void ecp_st_smooth_move::conditional_execution(){
	std::cout << "CONDITIONAL EXECUTION" << std::endl;
	bcl_smooth->Move();
}

}

}

}

}

