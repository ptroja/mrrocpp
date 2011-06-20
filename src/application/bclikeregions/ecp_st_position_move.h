/**
 * \file ecp_st_position_move.h
 * \brief Position move subtask class
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#ifndef ECP_ST_POSITION_MOVE_H_
#define ECP_ST_POSITION_MOVE_H_

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_sub_task.h"
#include "bcl_types.h"
#include <boost/shared_ptr.hpp>
#include "ecp_mp_message.h"
#include "../../generator/ecp/ecp_g_newsmooth.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task{

class bcl_t_switcher;

}

namespace sub_task {



class ecp_st_position_move : public sub_task {
public:
	/**
	 * Subtask constructor
	 * @param _ecp_t reference to parent task
	 */
	ecp_st_position_move(task::task & _ecp_t);
	/**
	 * Class destructor
	 */
	virtual ~ecp_st_position_move();
	/**
	 * Method called by parent task to start execution of subtask
	 */
	void conditional_execution();

private:
	shared_ptr<generator::newsmooth> smooth;
	task::bcl_t_switcher & bcl_ecp;

	ecp_mp_message msg;
};

}

}

}

}

#endif /* ECP_ST_SMOOTH_MOVE_H_ */
