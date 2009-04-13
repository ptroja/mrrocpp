#if !defined(_ECP_T_FR_IRP6OT_H)
#define _ECP_T_FR_IRP6OT_H

#include "ecp/common/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class fr: public common::task::base  {
protected:
	trajectory_description tdes_joint;
	common::generator::linear_parabolic *adg1;
	// parabolic_generator adg1(JOINT, 20., joint_pp);   // generator dla trajektorii dojscia we wsp. wew
	// generator dla trajektorii dojscia we wsp. zew.
	common::generator::linear_parabolic *adg2;
	common::generator::elipsoid *el;
	double ta[MAX_SERVOS_NR];
	double tb[MAX_SERVOS_NR];

public:
	// KONSTRUKTORY
	fr(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
