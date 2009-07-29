#if !defined(_ECP_T_HAAR_IRP6OT_H)
#define _ECP_T_HAAR_IRP6OT_H



#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "ecp/common/ecp_task.h"


#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_planar_irp6ot.h"
#include "ecp/irp6_on_track/ecp_g_rotate_gripper.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

#include "ecp_mp/ecp_mp_s_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class haar: public common::task::task  {

	bool rotation;
	std::string smooth_path;
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
	lib::trajectory_description td;

	 ecp_g_rotate_gripper* rot_gripper_gen;

public:
	//Konstruktory.
	haar(lib::configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void task_initialization(void);
	void main_task_algorithm(void);
	//Metods modifing td.
	void set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6);
	void init_td(lib::POSE_SPECIFICATION ps, int internode_no);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
