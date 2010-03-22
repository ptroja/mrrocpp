#if !defined(_ECP_T_RCSC_IRP6OT_H)
#define _ECP_T_RCSC_IRP6OT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_transparent.h"
#include "ecp/common/task/ecp_st_go.h"
#include "ecp_mp/sensor/ecp_mp_s_fradia_sensor.h"
#include "../servovision/ecp_g_ib_eih.h"
#include "../servovision/visual_servo_regulator_p.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class rcsc: public common::task::task
{
protected:
    //generatory
	common::generator::transparent* gt;
	common::generator::tff_nose_run* nrg;
	common::generator::tff_rubik_grab* rgg;
	common::generator::tff_gripper_approach* gag;
	common::generator::tff_rubik_face_rotate* rfrg;
	common::generator::teach_in* tig;
	common::generator::smooth* sg;
	common::generator::ecp_g_ib_eih* ib_eih;
	common::generator::bias_edp_force* befg;
	common::generator::weight_meassure* wmg;
    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

	ecp_mp::sensor::fradia_sensor <ecp::common::generator::visual_object_tracker>* vsp_fradia;
	ecp::common::generator::regulator_p <4, 4>* regulator;

public:
    rcsc(lib::configurator &_config);
    ~rcsc();

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
