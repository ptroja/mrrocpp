/*
 * bclike_smooth.h
 *
 *  Created on: 05-07-2010
 *      Author: kszkudla
 */

#ifndef BCLIKE_SMOOTH_H_
#define BCLIKE_SMOOTH_H_

#include "../../generator/ecp/ecp_g_smooth.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "bcl_types.h"


namespace mrrocpp {

namespace ecp {

namespace common {

namespace task{
class bclikeregions_task;
}

namespace generator {

class bclike_smooth: public mrrocpp::ecp::common::generator::smooth {
public:
	bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task, bool synchronized = true);
	bclike_smooth(mrrocpp::ecp::common::task::bclikeregions_task & task, bool synchronized = true);

	virtual ~bclike_smooth();

	virtual bool next_step();
	virtual bool first_step();

private:
//	boost::shared_ptr <ecp_mp::sensor::sensor_interface> vsp_fradia;
//	boost::shared_ptr <ecp_mp::sensor::fradia_sensor> vsp_fradia;
	boost::shared_ptr<task::bcl_fradia_sensor> vsp_fradia;
};

}

}

}

}

#endif /* BCLIKE_SMOOTH_H_ */
