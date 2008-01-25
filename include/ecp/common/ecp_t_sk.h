// -------------------------------------------------------------------------
//                            ecp_t_sk.h
// 
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(_ECP_T_SK_H)
#define _ECP_T_SK_H

#include "ecp/common/ecp_task.h"

class ecp_task_sk: public ecp_task  {
protected:
	ecp_tff_nose_run_generator* nrg;
	y_edge_follow_force_generator* yefg;
	bool save_activated;
	
public:
	// KONSTRUKTORY
	ecp_task_sk();
	~ecp_task_sk();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

#endif
