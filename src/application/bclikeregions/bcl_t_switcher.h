/*
 * bcl_t_switcher.h
 *
 *  Created on: Jul 6, 2010
 *      Author: kszkudla
 */

#ifndef BCL_T_SWITCHER_H_
#define BCL_T_SWITCHER_H_

#include "base/ecp/ecp_task.h"
#include "bclike_smooth.h"
#include <boost/shared_ptr.hpp>
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp_mp_bclike.h"
#include "ecp_st_smooth_move.h"
#include "bcl_message.h"

#include "ecp_mp_st_smooth_move.h"

#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

//const std::string BCL_MOTION_DIR_STR = "BCL_MOTION_DIR";

enum BCL_MOTION_DIR{
	LEFT, RIGHT, START
};

class bcl_t_switcher: public mrrocpp::ecp::common::task::task {
public:
	bcl_t_switcher(lib::configurator &_config);
	virtual ~bcl_t_switcher();

    void mp_2_ecp_next_state_string_handler(void);

//	virtual boost::shared_ptr <bcl_fradia_sensor> get_vsp_fradia();

	virtual bcl_fradia_sensor* get_vsp_fradia();

//	shared_ptr<bcl_fradia_sensor> vsp_fradia;
	bcl_fradia_sensor* vsp_fradia;

	void addReading(regions& reg);
	regions getReading();

private:
//	  generator::constant_velocity* cvgenjoint;; -- generator o stalej predkosci
//    shared_ptr<generator::smooth> bc_smooth;

//	shared_ptr<generator::bclike_smooth> bc_smooth;

	shared_ptr<generator::newsmooth> bc_smooth;

	std::vector<double> vec;

	std::vector<regions> reading;

	bcl_message msg;

//	shared_ptr<ecp_sub_task> bcl_recognition;
//	shared_ptr<ecp_sub_task> bc_read;
};
}

}

}

}

#endif /* BCL_T_SWITCHER_H_ */
