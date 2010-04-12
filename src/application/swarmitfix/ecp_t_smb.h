#if !defined(_ECP_T_SMB_SWARMITFIX_H)
#define _ECP_T_SMB_SWARMITFIX_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_transparent.h"


namespace mrrocpp {
namespace ecp {
namespace smb {
namespace task {

class swarmitfix: public common::task::task
{
protected:
    //generatory
	common::generator::transparent* gt;
	common::generator::smooth* sg;
	common::generator::sleep* g_sleep;
	common::generator::epos* g_epos;
	generator::pin_lock* g_pin_lock;
	generator::pin_unlock* g_pin_unlock;
	generator::pin_rise* g_pin_rise;
	generator::pin_lower* g_pin_lower;

public:
    // KONSTRUKTORY
    swarmitfix(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif