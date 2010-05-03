/*
 * generator/ecp_g_head.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SHEAD_H_
#define ECP_G_SHEAD_H_

#include <time.h>
#include "ecp/common/generator/ecp_generator.h"
#include "lib/data_port_headers/shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

class head_soldify: public common::generator::generator {
private:

public:
	head_soldify(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

class head_desoldify: public common::generator::generator {
private:

public:
	head_desoldify(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

class head_vacuum_on: public common::generator::generator {
private:

public:
	head_vacuum_on(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation
};

class head_vacuum_off: public common::generator::generator {
private:

public:
	head_vacuum_off(common::task::task& _ecp_task); //constructor
	bool first_step(); //first step generation
	bool next_step(); //next step generation

};

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_SLEEP_H_ */
