#if !defined(_ECP_T_FR_IRP6OT_H)
#define _ECP_T_FR_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_task_fr_irp6ot: public common::task::ecp_task  {
protected:
	trajectory_description tdes_joint;
	common::generator::ecp_linear_parabolic_generator *adg1;
	// parabolic_generator adg1(JOINT, 20., joint_pp);   // generator dla trajektorii dojscia we wsp. wew
	// generator dla trajektorii dojscia we wsp. zew.
	common::generator::ecp_linear_parabolic_generator *adg2;
	common::generator::ecp_elipsoid_generator *el;
	double ta[MAX_SERVOS_NR];
	double tb[MAX_SERVOS_NR];

public:
	// KONSTRUKTORY
	ecp_task_fr_irp6ot(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
