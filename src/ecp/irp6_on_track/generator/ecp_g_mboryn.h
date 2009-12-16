/*
 * generator/ecp_g_mboryn.h
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#ifndef ECP_G_MBORYN_H_
#define ECP_G_MBORYN_H_

#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

class ecp_g_mboryn: public mrrocpp::ecp::common::generator::generator {
public:
	ecp_g_mboryn(common::task::task & _ecp_task);
	virtual ~ecp_g_mboryn();
	virtual bool first_step();
	virtual bool next_step();
};

} // namespace generator

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_MBORYN_H_ */
