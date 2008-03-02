#if !defined(_ECP_T_DUNG_H)
#define _ECP_T_DUNG_H

#include "ecp/common/ecp_task.h"

class ecp_task_dung: public ecp_task  {

public:
	// KONSTRUKTORY
	ecp_task_dung(configurator &_config);
	~ecp_task_dung();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
