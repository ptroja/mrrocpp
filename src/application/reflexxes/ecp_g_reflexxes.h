#if !defined(_ECP_GEN_REFLEXXES_H)
#define _ECP_GEN_REFLEXXES_H

#include "base/ecp/ecp_generator.h"

#include "contrib/Reflexxes/include/ReflexxesAPI.h"
#include "contrib/Reflexxes/include/RMLPositionInputParameters.h"
#include "contrib/Reflexxes/include/RMLPositionOutputParameters.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class reflexxes : public common::generator::generator
{
private:
	//! Reflexxes API object.
	ReflexxesAPI *RML;

	//! OTG input parameters.
	RMLPositionInputParameters *IP;

	//! OTG output parameters.
	RMLPositionOutputParameters *OP;

	//! OTG flags.
	RMLPositionFlags Flags;

	//! ORG algorithm update result.
	int ResultValue;

	double goal[lib::MAX_SERVOS_NR];

	//! Display OTG results.
	void showOTG(int verbose) const;

	//! Do we have a fresh current position?
	bool fresh;

	//! Display results.
	bool display;

public:
	//! Constructor.
	reflexxes(common::task::task& _ecp_task);

	//! Destructor.
	~reflexxes();

	//! Set goal pose.
	void set_goal_pose(const double goal_joint_coordinates[lib::MAX_SERVOS_NR]);

	//! First step.
	virtual bool first_step();

	//! Next step.
	virtual bool next_step();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_GEN_REFLEXXES_H */
