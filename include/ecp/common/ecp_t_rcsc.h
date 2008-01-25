// -------------------------------------------------------------------------
//                            ecp_t_rcsc.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_TASK_RCSC_H)
#define _ECP_TASK_RCSC_H

#include "ecp/common/ecp_task.h"

void ecp_gripper_opening (ecp_task& ecp_object, double gripper_increment, int motion_time);

#endif
