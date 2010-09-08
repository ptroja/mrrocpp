/*
 * ecp_st_smooth_move.h
 *
 *  Created on: 20-07-2010
 *      Author: kszkudla
 */

#ifndef ECP_ST_POSITION_MOVE_H_
#define ECP_ST_POSITION_MOVE_H_

#include "base/ecp/ecp_task.h"
#include "bcl_types.h"
#include <boost/shared_ptr.hpp>
#include "ecp_mp_message.h"
#include "../../generator/ecp/ecp_g_newsmooth.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {


class bcl_t_switcher;

class ecp_st_position_move : public ecp_sub_task {
public:
	ecp_st_position_move(task & _ecp_t);
	virtual ~ecp_st_position_move();

	void conditional_execution();

private:
//	shared_ptr<generator::bclike_smooth> bcl_smooth;
	shared_ptr<generator::newsmooth> smooth;
	bcl_t_switcher & bcl_ecp;

	ecp_mp_message msg;
};

}

}

}

}

#endif /* ECP_ST_SMOOTH_MOVE_H_ */
