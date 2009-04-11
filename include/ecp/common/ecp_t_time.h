#if !defined(_ECP_T_SK_H)
#define _ECP_T_SK_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_time.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_task_time: public ecp_task  {
protected:
	time_generator *tfg;

public:
	// KONSTRUKTORY
	ecp_task_time(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);

};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_SK_H */
