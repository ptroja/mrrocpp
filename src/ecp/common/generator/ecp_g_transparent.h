#if !defined(_ECP_GENERATOR_T_H)
#define  _ECP_GENERATOR_T_H

#include "ecp/common/generator/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// Generator przezroczysty
class transparent : public common::generator::generator {

public:
	transparent(common::task::task& _ecp_task);
	
	bool throw_kinematics_exceptions;
	
	
	bool first_step ();
	bool next_step ();

	void execute_motion (void);
	
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GENERATOR_T_H */
