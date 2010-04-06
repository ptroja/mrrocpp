#if !defined(_ECP_T_SMB_SHEAD_H)
#define _ECP_T_SMB_SHEAD_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_transparent.h"


namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

class swarmitfix: public common::task::task
{
protected:
    //generatory
	common::generator::transparent* gt;
	common::generator::smooth* sg;

public:
    // KONSTRUKTORY
    swarmitfix(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif
