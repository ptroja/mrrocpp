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
#include "ecp/common/task/ecp_st_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

ecp_sub_task_tff_nose_run::ecp_sub_task_tff_nose_run(task &_ecp_t) :
	ecp_sub_task(_ecp_t)
{
	nrg = new generator::tff_nose_run(_ecp_t, 8);
}

void ecp_sub_task_tff_nose_run::conditional_execution()
{

	nrg->Move();
}

} // namespace task

} // namespace common
} // namespace ecp
} // namespace mrrocpp
