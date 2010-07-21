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
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "bcl_types.h"


namespace mrrocpp {

namespace ecp {

namespace common {

namespace task{
class bclikeregions_task;
class bcl_t_switcher;
}

namespace generator {

class bclike_smooth: public mrrocpp::ecp::common::generator::newsmooth {
public:
	bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task);
	bclike_smooth(mrrocpp::ecp::common::task::bclikeregions_task & task);

	virtual ~bclike_smooth();

	virtual bool next_step();
	virtual bool first_step();

private:
	task::regions reading;
	bool no_fradia;
	task::bcl_t_switcher & bcl_ecp;

//	task::bcl_fradia_sensor *vsp_fradia;

//	boost::shared_ptr <ecp_mp::sensor::sensor_interface> vsp_fradia;
//	boost::shared_ptr <ecp_mp::sensor::fradia_sensor> vsp_fradia;
	boost::shared_ptr<task::bcl_fradia_sensor> vsp_fradia;
};

}

}

}

}

#endif /* BCLIKE_SMOOTH_H_ */
