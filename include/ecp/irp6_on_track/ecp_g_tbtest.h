/*
 * ecp_g_tbtest.h
 *
 *  Created on: Jan 30, 2009
 *      Author: tbem
 */

#ifndef ECP_G_TBTEST_H_
#define ECP_G_TBTEST_H_

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"
#include "common/impconst.h"
#include "common/com_buf.h"
//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

class ecp_g_tbtest : public common::generator::base
{
	//Buffer used to communicate between ECP and VSP
//	ECP_VSP_MSG communication_buffer;
//	ecp_mp_cvfradia_sensor * sensor;
	//Object coordinates
//	SENSOR_IMAGE object_descriptor;

    trajectory_description td;
    double next_position[8];

public:
    ecp_g_tbtest (common::task::ecp_task& _ecp_task);
    virtual bool first_step();
    virtual bool next_step();
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_TBTEST_H_ */
