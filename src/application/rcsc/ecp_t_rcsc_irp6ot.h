#if !defined(_ECP_T_RCSC_IRP6OT_H)
#define _ECP_T_RCSC_IRP6OT_H

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"
#include "subtask/ecp_st_go.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"

#include "../servovision/simple_visual_servo_manager.h"
#include "../servovision/ib_eih_visual_servo.h"
#include "../servovision/cubic_constraint.h"
#include "../servovision/visual_servo_regulator_p.h"
#include "../servovision/object_reached_termination_condition.h"

using mrrocpp::ecp::common::generator::simple_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo;
using mrrocpp::ecp::common::generator::visual_servo_regulator;
using mrrocpp::ecp::common::generator::cubic_constraint;
using mrrocpp::ecp::common::generator::position_constraint;
using mrrocpp::ecp::common::generator::ib_eih_visual_servo;
using mrrocpp::ecp::common::generator::regulator_p;
using mrrocpp::ecp::common::generator::object_reached_termination_condition;
using mrrocpp::ecp::common::generator::termination_condition;
using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

class rcsc: public common::task::task
{
protected:
    //generatory
	shared_ptr<common::generator::transparent> gt;
	shared_ptr<common::generator::tff_rubik_grab> rgg;
	shared_ptr<common::generator::tff_gripper_approach> gag;
	shared_ptr<common::generator::tff_rubik_face_rotate> rfrg;
	shared_ptr<common::generator::teach_in> tig;
	shared_ptr<common::generator::smooth> sg;

	shared_ptr<common::generator::weight_meassure> wmg;
    //podzadania
	shared_ptr<common::task::ecp_sub_task_gripper_opening> go_st;

	shared_ptr<simple_visual_servo_manager> sm;
	shared_ptr<visual_servo> vs;
	shared_ptr<visual_servo_regulator> reg;
	shared_ptr<termination_condition> term_cond;
public:
    rcsc(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
