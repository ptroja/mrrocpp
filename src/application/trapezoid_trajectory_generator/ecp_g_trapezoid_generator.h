/*
 * ecp_g_trapezoid_generator.h
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#ifndef ECP_G_TRAPEZOID_GENERATOR_H_
#define ECP_G_TRAPEZOID_GENERATOR_H_

#include "base/ecp/ecp_generator.h"

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
};

}

}

}

#endif /* ECP_G_TRAPEZOID_GENERATOR_H_ */
