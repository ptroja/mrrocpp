#if !defined(_ECP_T_POURING_IRP6P_H)
#define _ECP_T_POURING_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_generator_t.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

class ecp_task_pouring_irp6p: public common::ecp_task
{
protected:
	common::ecp_smooth_generator* sg;
    //podzadania
	common::ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    ecp_task_pouring_irp6p(configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);

};

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
