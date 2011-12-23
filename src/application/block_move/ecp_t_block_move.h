#if !defined(_ECP_T_BLOCK_MOVE_H)
#define _ECP_T_BLOCK_MOVE_H

#include <cstdio>
#include <string>

#include <boost/shared_ptr.hpp>

#include "base/lib/logger.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "sensor/discode/discode_sensor.h"

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_mp_st_smooth_file_from_mp.h"
#include "subtask/ecp_st_go.h"

#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_g_get_position.h"

#include "../visual_servoing/visual_servo.h"
#include "../visual_servoing/single_visual_servo_manager.h"
#include "../visual_servoing/ib_eih_visual_servo.h"
#include "../visual_servoing/visual_servo_regulator_p.h"
#include "../visual_servoing/cubic_constraint.h"
#include "../visual_servoing/object_reached_termination_condition.h"
#include "../visual_servoing/timeout_termination_condition.h"
#include "../visual_servoing/IBReading.h"

#include "../visual_servoing/visual_servoing.h"
#include "../visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"

using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using boost::shared_ptr;

using namespace mrrocpp::ecp::servovision;
using namespace mrrocpp::ecp_mp::sensor::discode;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class block_move : public common::task::task
{
protected:

	common::generator::tff_gripper_approach* gtga;
	common::generator::newsmooth* sg;
	common::generator::get_position* gp;

	//common::sub_task::gripper_opening* stgo;

	shared_ptr<single_visual_servo_manager> /*sm1,*/ sm2;
	shared_ptr<visual_servo> /*vs1,*/ vs2;
	shared_ptr<visual_servo_regulator> /*reg1,*/ reg2;
	shared_ptr<discode_sensor> ds_rpc, /*ds1, */ds2;

	shared_ptr<termination_condition> /*object_reached_term_cond1, */object_reached_term_cond2;
	shared_ptr<termination_condition> /*timeout_term_cond1, */timeout_term_cond2;

	std::string ds_config_section_name;
	std::string /*vs_config_section_name1,*/ vs_config_section_name2;

	double position[6];

public:

	block_move(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
