#if !defined(_ECP_T_PW_SCENA_IRP6OT_H)
#define _ECP_T_PW_SCENA_IRP6OT_H



#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "ecp/common/ecp_task.h"


#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_pw_scena.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_planar_irp6ot.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class pw_scena: public common::task::base  {

	generator::pw_scena* scena_gen;
	//Smoth movement generator
	common::generator::smooth* smooth_gen;
	//Calibration of force
	common::generator::bias_edp_force* bef_gen;
	//Gripper approach with force control
	common::generator::tff_gripper_approach* ga_gen;
	//Linear generator
	common::generator::linear* linear_gen;
	//Planar servoing.
	ecp_vis_ib_eih_planar_irp6ot* planar_vis;
	//Trajectory description.
	trajectory_description td;

public:
	//Konstruktory.
	pw_scena(configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void task_initialization(void);
	void main_task_algorithm(void);
	//Metods modifing td.
	void set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6);
	void init_td(POSE_SPECIFICATION ps, int internode_no);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
