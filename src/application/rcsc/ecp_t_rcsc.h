#if !defined(_ECP_T_RCSC_H)
#define _ECP_T_RCSC_H

#include <boost/shared_ptr.hpp>

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"
#include "generator/ecp/force/ecp_g_weight_measure.h"

#include "generator/ecp/force/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "generator/ecp/ecp_g_newsmooth.h"
/*
 #include "sensor/fradia/ecp_mp_s_fradia_sensor.h"

 #include "../servovision/single_visual_servo_manager.h"
 #include "../servovision/ib_eih_visual_servo.h"
 #include "../servovision/cubic_constraint.h"
 #include "../servovision/visual_servo_regulator_p.h"
 #include "../servovision/object_reached_termination_condition.h"

 using mrrocpp::ecp::common::generator::single_visual_servo_manager;
 using mrrocpp::ecp::common::generator::visual_servo_manager;
 using namespace mrrocpp::ecp::servovision;
 */
using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class rcsc : public common::task::task
{
protected:
	//generatory
	generator::transparent* gt;

	generator::tff_gripper_approach* gag;
	generator::tff_rubik_face_rotate* rfrg;
	generator::teach_in* tig;
	generator::newsmooth* sg;
	generator::newsmooth* sgaa;

	/*
	 shared_ptr <single_visual_servo_manager> sm;
	 shared_ptr <visual_servo> vs;
	 shared_ptr <visual_servo_regulator> reg;
	 shared_ptr <termination_condition> term_cond;
	 */
public:
	rcsc(lib::configurator &_config);
	~rcsc();

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
