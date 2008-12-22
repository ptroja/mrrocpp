#if !defined(_ECP_T_TEST_IRP6P_H)
#define _ECP_T_TEST_IRP6P_H

#include "ecp/common/ecp_task.h"

class ecp_task_test_irp6p: public ecp_task  {

public:
	// KONSTRUKTORY
	ecp_task_test_irp6p(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);

};

#endif
