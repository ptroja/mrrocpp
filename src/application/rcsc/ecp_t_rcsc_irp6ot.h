#if !defined(_ECP_T_RCSC_IRP6OT_H)
#define _ECP_T_RCSC_IRP6OT_H

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"
#include "generator/ecp/force/ecp_g_weight_measure.h"
#include "generator/ecp/force/ecp_g_tff_rubik_grab.h"
#include "generator/ecp/force/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "subtask/ecp_st_go.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"

#include "../servovision/single_visual_servo_manager.h"
#include "../servovision/ib_eih_visual_servo.h"
#include "../servovision/cubic_constraint.h"
#include "../servovision/visual_servo_regulator_p.h"
#include "../servovision/object_reached_termination_condition.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;
using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

class rcsc : public common::task::task
{
protected:
	//generatory
	common::generator::transparent* gt;

	common::generator::tff_rubik_grab* rgg;
	common::generator::tff_gripper_approach* gag;
	common::generator::tff_rubik_face_rotate* rfrg;
	common::generator::teach_in* tig;
	//common::generator::smooth* sg;

	common::generator::weight_measure* wmg;
	//podzadania
	common::sub_task::gripper_opening* go_st;

	shared_ptr<single_visual_servo_manager> sm;
	shared_ptr<visual_servo> vs;
	shared_ptr<visual_servo_regulator> reg;
	shared_ptr<termination_condition> term_cond;
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
