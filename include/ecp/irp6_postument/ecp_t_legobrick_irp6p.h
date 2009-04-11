#if !defined(_ECP_T_LEGOBRICK_IRP6P_H)
#define _ECP_T_LEGOBRICK_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_task_lego_brick_irp6p: public ecp_task
{
protected:
    //generatory
    ecp_smooth_generator* sg;
    bias_edp_force_generator* befg;
    legobrick_attach_force_generator* afg;
    legobrick_detach_force_generator* dfg;

public:
    ecp_task_lego_brick_irp6p(configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
