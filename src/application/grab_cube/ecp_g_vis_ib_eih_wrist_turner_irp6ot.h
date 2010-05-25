/*
 * generator/ecp_g_vis_ib_eih_wrist_turner_irp6ot.h
 *
 *  Created on: DEC 10, 2009
 *      Author: rtulwin
 */

#ifndef ECP_VIS_IB_EIH_WRIST_TURNER_IRP6OT_H_
#define ECP_VIS_IB_EIH_WRIST_TURNER_IRP6OT_H_

#include "ecp/common/generator/ecp_g_visual_servo.h"

#include <string.h>
#include <math.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"


#include <iostream>

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

#define MOTION_STEPS 25

class ecp_vis_ib_eih_wrist_turner_irp6ot: public common::generator::ecp_visual_servo {

public:

	lib::sensor *vsp_fradia; //wirtualny sensor
    double next_position[MAX_SERVOS_NR]; 	//pozycja w nastepnym kroku.
    bool tracking; //jesli true, obiekt jest sledzony, jesli false, zagubiony (przychodzi z fradii)
    bool reached; //jesli true, pozycja nadgarstka jest odpowiednia do chwycenia (przychodzi z fradii)

	ecp_vis_ib_eih_wrist_turner_irp6ot(common::task::task& _ecp_task);
	virtual bool first_step(void);
	virtual bool next_step_without_constraints();
	virtual void limit_step();

};

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_VIS_IB_EIH_WRIST_TURNER_IRP6OT_H_ */
