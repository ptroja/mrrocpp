#if !defined(_ECP_T_PLAYERPOS_H)
#define _ECP_T_PLAYERPOS_H

#include "ecp/common/ecp_task.h"

class ecp_task_playerpos: public ecp_task  {
protected:
	playerpos_generator* ppg;

public:
	// KONSTRUKTORY
	ecp_task_playerpos(configurator &_config);
	~ecp_task_playerpos();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

#endif
