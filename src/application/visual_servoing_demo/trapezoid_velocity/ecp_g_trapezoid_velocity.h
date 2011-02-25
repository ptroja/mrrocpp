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

class trapezoid_velocity_gen: public mrrocpp::ecp::common::generator::generator {
public:
	trapezoid_velocity_gen(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name);
	virtual ~trapezoid_velocity_gen();
	virtual bool first_step();
	virtual bool next_step();
private:

};

}

}

}

}

#endif /* TRAPEZOID_VELOCITY_H_ */
