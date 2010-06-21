#if !defined(_ECP_T_FESTIVAL_H)
#define _ECP_T_FESTIVAL_H

#include "base/ecp/ecp_task.h"
#include "robot/festival/ecp_g_festival.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace task {

class task: public common::task::task  {
protected:
	generator::generator fg;

public:
	task(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace festival
} // namespace ecp
} // namespace mrrocpp

#endif
