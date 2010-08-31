/*
 * bclike_smooth.h
 *
 *  Created on: 05-07-2010
 *      Author: kszkudla
 */

#ifndef BCLIKE_SMOOTH_H_
#define BCLIKE_SMOOTH_H_

#include "../../generator/ecp/ecp_g_newsmooth.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "generator/ecp/ecp_g_constant_velocity.h"
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "bcl_types.h"
#include "bcl_message.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task{
class bclikeregions_task;
class bcl_t_switcher;
}

namespace generator {

//class bclike_smooth: public mrrocpp::ecp::common::generator::newsmooth {
class bclike_smooth: public mrrocpp::ecp::common::generator::constant_velocity {
public:
	bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task);
	bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task);
	bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task, task::bcl_fradia_sensor* fr);

	virtual ~bclike_smooth();

	virtual bool next_step();
	virtual bool first_step();

private:
	task::fradia_regions reading;
	std::vector<std::pair<task::mrrocpp_regions, bool> > readings;
	bool no_fradia;
	task::bcl_t_switcher & bcl_ecp;

	bcl_message msg;

	task::bcl_fradia_sensor* vsp_fradia;

	int num_send;

	lib::Homog_matrix actual_pos;
	lib::Homog_matrix tmp_pos;

	void translateToRobotPosition(task::fradia_regions& regs);
	void addCodesToVector(task::fradia_regions reading);
	bool checkIfCodeBeenRead(task::mrrocpp_regions& code);
	bool codesIntersect(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2);
	bool sendNextPart();
};

}

}

}

}

#endif /* BCLIKE_SMOOTH_H_ */
