/*
 * task/ecp_t_mboryn.h
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_T_MBORYN_H_
#define ECP_T_MBORYN_H_

#include "ecp/common/task/ecp_task.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp_g_mboryn.h"
#include "ecp_mp/sensor/ecp_mp_s_cvfradia.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace task {

class ecp_t_mboryn: public mrrocpp::ecp::common::task::task {
public:
	ecp_t_mboryn(mrrocpp::lib::configurator& _configurator);
	virtual ~ecp_t_mboryn();
	void main_task_algorithm(void);
protected:
	generator::ecp_g_mboryn* ecp_g_mboryn_;
	lib::sensor *vsp_fradia;
	mrrocpp::ecp::common::generator::smooth* smooth_gen;

	/** Moves (using smooth generator) to initial position. */
	void moveToInitialPosition();

	/*static const double initialPositionJoints[MAX_SERVOS_NR] = {
			0, -0.013, -1.442, -0.275, 0.01, 4.686, -0.070, 0.090
	};*/
	static const double initialPositionJoints[MAX_SERVOS_NR];
};

}

}

}

}

#endif /* ECP_T_MBORYN_H_ */
