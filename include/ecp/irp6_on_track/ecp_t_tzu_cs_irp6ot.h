#if !defined(_ECP_T_TZU_CS_IRP6OT_H)
#define _ECP_T_TZU_CS_IRP6OT_H

#include "ecp/common/ecp_task.h"

class ecp_task_tzu_cs_irp6ot :  public ecp_task  
{
public:
	ecp_task_tzu_cs_irp6ot(configurator &_config);
	~ecp_task_tzu_cs_irp6ot();
	
	// methods for ECP template to redefine in concrete classes
	/** metoda odpowiedzialna za inicjalizacje zmiennych zadania **/
	void task_initialization(void);
	/** metoda odpowiedzialna za wykonanie zadania **/
	void main_task_algorithm(void);
};

#endif

