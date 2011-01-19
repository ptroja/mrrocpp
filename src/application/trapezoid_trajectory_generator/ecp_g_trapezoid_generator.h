/*
 * ecp_g_trapezoid_generator.h
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#ifndef ECP_G_TRAPEZOID_GENERATOR_H_
#define ECP_G_TRAPEZOID_GENERATOR_H_

#include "base/ecp/ecp_generator.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace trapezoid {

class trapezoid_generator : public mrrocpp::ecp::common::generator::generator
{
public:
	trapezoid_generator(mrrocpp::ecp::common::task::task & ecp_task);
	virtual ~trapezoid_generator();

	bool first_step();
	bool next_step();

	void set_params(int axis_idx, double accel1, double accel2, double v_max, double time_v_max);
private:
	int motion_steps;
	bool current_theta_saved;
	double current_arm_coordinates[lib::MAX_SERVOS_NR];

	int axis_idx;
	double accel1;
	double accel2;
	double v_max;
	double time_v_max;
	enum {
		S_INIT, S_ACCEL, S_CONST_SPEED, S_SLOWDOWN, S_STOP
	}state;
	int steps_count;
	static const int STEPS_NUMBER_INIT_STOP=20;
	double s0;
	double s;

};

}

}

}

#endif /* ECP_G_TRAPEZOID_GENERATOR_H_ */
