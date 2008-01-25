// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
// 
// Modyfikacje:
// 1. metody wirtualne w klasie bazowej sensor - ok. 160
// 2. bonusy do testowania
// 
// Ostatnia modyfikacja: 25.06.2003
// autor modyfikacji: tkornuta
// -------------------------------------------------------------------------

#if !defined(_ECP_T_TS_IRP6OT_H)
#define _ECP_T_TS_IRP6OT_H

#include "ecp/common/ecp_task.h"

class ecp_task_ts_irp6ot: public ecp_task  {

public:
	// KONSTRUKTORY
	ecp_task_ts_irp6ot();
	~ecp_task_ts_irp6ot();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
