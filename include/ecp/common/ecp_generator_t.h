#if !defined(_ECP_GENERATOR_T_H)
#define  _ECP_GENERATOR_T_H

#include "ecp/common/ecp_generator.h"

// Generator przezroczysty
class ecp_generator_t : public ecp_generator {

public:
	ecp_generator_t(ecp_task& _ecp_task);
	
	bool throw_kinematics_exceptions;
	
	
	bool first_step ();
	bool next_step ();

	void execute_motion (void);
	
};

#endif /* _ECP_GENERATOR_T_H */
