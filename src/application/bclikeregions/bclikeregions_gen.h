/*
 * bclikeregions_gen.h
 *
 *  Created on: May 18, 2010
 *      Author: kszkudla
 */

#ifndef BCLIKEREGIONS_GEN_H_
#define BCLIKEREGIONS_GEN_H_

#include "base/ecp/ecp_generator.h"
#include "lib/mrmath/mrmath.h"
#include "../servovision/cubic_constraint.h"
#include "../servovision/position_constraint.h"

namespace mrrocpp{

namespace ecp {

namespace common {

namespace generator {

class bclikeregions_gen : public mrrocpp::ecp::common::generator::generator
{
public:
	bclikeregions_gen(mrrocpp::ecp::common::task::task & ecp_task);
	virtual ~bclikeregions_gen();
	virtual bool first_step();
	virtual bool next_step();
	int motion_steps;
private:
	void countRotationMatrix(lib::Homog_matrix &mat, double x_angle, double y_angle, double z_angle);
	void countTranslationMatrix(lib::Homog_matrix &mat, double x, double y, double z);

	//boost::shared_ptr <cp_mp::sensor::sensor_interface> vsp_fradia;

	double angle;

	const double r;

	bool first_pos_saved;
	bool on_position;

	lib::Homog_matrix first_pos;
	lib::Homog_matrix actual_pos;
	lib::Homog_matrix prev_pos;
	lib::Homog_matrix next_pos;
};

}

}

}

}

#endif /* BCLIKEREGIONS_GEN_H_ */
