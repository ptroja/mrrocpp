#if !defined(_ECP_T_DUNG_H)
#define _ECP_T_DUNG_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

class dung: public common::task::task {

public:
	// KONSTRUKTORY
	dung(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
