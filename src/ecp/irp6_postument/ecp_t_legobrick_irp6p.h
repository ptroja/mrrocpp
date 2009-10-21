#if !defined(_ECP_T_LEGOBRICK_IRP6P_H)
#define _ECP_T_LEGOBRICK_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class lego_brick: public common::task::task
{
protected:
    //generatory
	common::generator::smooth* sg;
	common::generator::bias_edp_force* befg;
	common::generator::legobrick_attach_force* afg;
	common::generator::legobrick_detach_force* dfg;

public:
    lego_brick(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
