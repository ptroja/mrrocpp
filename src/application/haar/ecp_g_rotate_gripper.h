/*
 * generator/ecp_g_vis_ib_eih_planar_irp6ot.h
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#ifndef ECP_G_ROTATE_GRIPPER_H_
#define  ECP_G_ROTATE_GRIPPER_H_

#include "ecp/common/generator/ecp_generator.h"

#include <string.h>
#include <math.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"


#include <iostream>

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {



class ecp_g_rotate_gripper: public common::generator::generator {
	//Wirtualny sensor
	lib::sensor *vsp_fradia;
	double speed;
	lib::trajectory_description td;
	bool lastStep;
//	FILE* research;

public:
	ecp_g_rotate_gripper(common::task::task& _ecp_task, double speed);
	virtual ~ecp_g_rotate_gripper();
	virtual bool first_step(void);
	virtual bool next_step();
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* generator/ecp_g_rotate_gripper_H_ */


