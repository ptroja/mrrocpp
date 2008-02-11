#if !defined(_ECP_GENERATOR_T_H)
#define  _ECP_GENERATOR_T_H

#include "ecp/common/ecp_generator.h"

// Generator przezroczysty
class ecp_generator_t : public ecp_generator {

public:
	ecp_generator_t(ecp_task& _ecp_task, bool _is_robot_active);
	
	virtual bool first_step ();
	virtual bool next_step ();

};

#endif /* _ECP_GENERATOR_T_H */
