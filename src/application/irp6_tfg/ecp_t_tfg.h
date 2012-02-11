#if !defined(_ECP_T_TFG_H)
#define _ECP_T_TFG_H

#include "base/ecp/ecp_task.h"


namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace task {

class tfg : public common::task::task
{
protected:

	bool save_activated;

public:
	// KONSTRUKTORY
	tfg(lib::configurator &_config);

};

}
} // namespace irp6_tfg
} // namespace ecp
} // namespace mrrocpp

#endif
