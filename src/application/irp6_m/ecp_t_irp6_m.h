#if !defined(_ECP_T_IRP6_M_H)
#define _ECP_T_IRP6_M_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class irp6_m : public common::task::task
{
public:
	irp6_m(lib::configurator &_config);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
