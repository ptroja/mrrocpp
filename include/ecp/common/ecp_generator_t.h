#if !defined(_ECP_GENERATOR_T_H)
#define  _ECP_GENERATOR_T_H

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// Generator przezroczysty
class ecp_generator_t : public common::ecp_generator {

public:
	ecp_generator_t(ecp_task& _ecp_task);
	
	bool throw_kinematics_exceptions;
	
	
	bool first_step ();
	bool next_step ();

	void execute_motion (void);
	
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_T_H */
