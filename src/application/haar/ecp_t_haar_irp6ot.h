#if !defined(_ECP_T_HAAR_IRP6OT_H)
#define _ECP_T_HAAR_IRP6OT_H

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/ecp/ecp_task.h"

#include "base/lib/sr/srlib.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "generator/ecp/vision/ecp_g_vis_ib_eih_planar_irp6ot.h"
#include "generator/ecp/force/ecp_g_bias_edp_force.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "ecp_g_rotate_gripper.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_jarosz.h"


#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"

//#define JAW_PINCHING_0 -0.016//zacisk szczeki dla puszki
//#define JAW_PINCHING_1 -0.024//zacisk szczeki dla pudelka
//#define LOWERNIG_INTERVAL_0 -0.073 //interwal co ktory wlaczany jest serwomechanizm w plaszczyznie							//dla puszki
//#define LOWERNIG_INTERVAL_1 -0.51
//#define GAGEN_INTERVAL_0 500 // ustawienie generatora gripper approach
//#define GAGEN_INTERVAL_1 401

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

class haar : public common::task::task
{
	bool rotation;
	std::string smooth_path;
	int object_type;
	//Smoth movement generator
	common::generator::newsmooth* smooth_gen;
	//Calibration of force
	common::generator::bias_edp_force* bef_gen;
	//Gripper approach with force control
	common::generator::tff_gripper_approach* ga_gen;
	//Linear generator
	common::generator::linear* linear_gen;
	//Planar servoing.
	generator::ecp_vis_ib_eih_planar_irp6ot* planar_vis;
	//Trajectory description.
	lib::trajectory_description td;
	//Rotation generator.
	ecp_g_rotate_gripper* rot_gripper_gen;

	float jaw_pinching;
	float lowering_interval;
	int ga_gen_interval;

public:
	//Konstruktory.
	haar(lib::configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void main_task_algorithm(void);

	//Metods modifing td.
	void set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6);
	void init_td(lib::ECP_POSE_SPECIFICATION ps, int internode_no);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
