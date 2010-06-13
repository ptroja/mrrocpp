// -------------------------------------------------------------------------
//                            ecp_st_go.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
//
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "ecp/common/task/ecp_st_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_bias_edp_force::ecp_sub_task_bias_edp_force(task &_ecp_t) :
	ecp_sub_task(_ecp_t)
{
	befg = new generator::bias_edp_force(_ecp_t);
}

void ecp_sub_task_bias_edp_force::conditional_execution()
{

	befg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
