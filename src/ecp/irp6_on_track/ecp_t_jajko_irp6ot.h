#if !defined(_ECP_T_JAJKO_IRP6OT_H)
#define _ECP_T_JAJKO_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class jajko: public common::task::task  {
protected:
	common::generator::y_egg_force* yefg;

public:
	// KONSTRUKTORY
	jajko(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
