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

#if !defined(_ECP_T_FESTIVAL_H)
#define _ECP_T_FESTIVAL_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/common/ecp_t_rcsc.h"

class ecp_task_festival: public ecp_task  {
protected:
	ecp_festival_generator* fg;

public:
	// KONSTRUKTORY
	ecp_task_festival();
	~ecp_task_festival();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
