#if !defined(_ECP_T_SK_H)
#define _ECP_T_SK_H

#include "base/ecp/ecp_task.h"
#include "ecp_g_time.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class time: public common::task::task  {
protected:
	generator::time *tfg;

public:
	// KONSTRUKTORY
	time(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_SK_H */
