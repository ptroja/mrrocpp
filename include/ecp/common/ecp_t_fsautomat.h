// -------------------------------------------------------------------------
//                            ecp_t_fsautomat.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Ostatnia modyfikacja:	sierpien 2008
// Autor:						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_TASK_FSAUTOMAT_H)
#define _ECP_TASK_FSAUTOMAT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

void ecp_gripper_opening (common::task::ecp_task& ecp_object, double gripper_increment, int motion_time);

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
