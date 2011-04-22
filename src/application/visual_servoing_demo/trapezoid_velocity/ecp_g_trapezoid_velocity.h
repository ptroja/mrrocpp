/*
 * trapezoid_velocity.h
 *
 *  Created on: 25-02-2011
 *      Author: mateusz
 */

#ifndef TRAPEZOID_VELOCITY_H_
#define TRAPEZOID_VELOCITY_H_

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class trapezoid_velocity: public mrrocpp::ecp::common::generator::generator {
public:
	trapezoid_velocity(mrrocpp::ecp::common::task::task & ecp_task);
	virtual ~trapezoid_velocity();
	virtual bool first_step();
	virtual bool next_step();

	void set_params(int axis_idx, double accel1, double accel2, double v_max);
private:
	int motion_steps;
	double dt;
	bool current_theta_saved;
	double current_arm_coordinates[lib::MAX_SERVOS_NR];

	int axis_idx;
	double accel1;
	double accel2;
	double v_max;

	enum {
		S_INIT, S_ACCEL, S_CONST_SPEED, S_SLOWDOWN, S_STOP
	} state;
	int steps_count;
	static const int STEPS_NUMBER_INIT_STOP = 20;
	static const int STEPS_NUMBER_CONST_SPEED = 50;

	double s;
	double v;
};

}

}

}

}

#endif /* TRAPEZOID_VELOCITY_H_ */
