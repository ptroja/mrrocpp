#ifndef ECP_G_PCBIRD_NOSE_RUN_H_
#define ECP_G_PCBIRD_NOSE_RUN_H_

#include "generator/ecp/force/ecp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class pcbird_nose_run : public tff_nose_run
{
	int count;

public:
	// konstruktor
	pcbird_nose_run(common::task::task& _ecp_task, int step = 0);

	virtual bool first_step();
	virtual bool next_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

