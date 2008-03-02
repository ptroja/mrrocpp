#if !defined(_ECP_T_SPEECHRECOGNITION_H)
#define _ECP_T_SPEECHRECOGNITION_H

#include "ecp/common/ecp_task.h"
#include "ecp/player/ecp_g_speechrecognition.h"

class ecp_task_speechrecognition: public ecp_task  {
protected:
	speechrecognition_generator* srg;

public:
	// KONSTRUKTORY
	ecp_task_speechrecognition(configurator &_config);
	~ecp_task_speechrecognition();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif /* _ECP_T_SPEECHRECOGNITION_H */
