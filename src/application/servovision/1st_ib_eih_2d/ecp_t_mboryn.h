/*
 * $Id$
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_T_MBORYN_H_
#define ECP_T_MBORYN_H_

#include "ecp/common/task/ecp_task.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_g_mboryn.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot_m {

namespace task {

/** @addtogroup servovision Visual servomechanism implementation
 *  @ingroup application
 *  Simple visual servo - blue ball follower
 *  @{
 */

/**
 *
 */
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

/** @} */// ecp_t_mboryn

} // namespace task

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_T_MBORYN_H_ */
