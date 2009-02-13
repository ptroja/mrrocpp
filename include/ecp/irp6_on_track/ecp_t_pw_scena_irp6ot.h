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

class ecp_task_pw_scena_irp6ot: public ecp_task  {

	ecp_g_pw_scena* scena_gen;
	//Smoth movement generator
	ecp_smooth_generator* smooth_gen;
	//Calibration of force
	bias_edp_force_generator* bef_gen;
	//Gripper approach with force control
	ecp_tff_gripper_approach_generator* ga_gen;
	//Linear generator
	ecp_linear_generator* linear_gen;
	//Planar servoing.
	ecp_vis_ib_eih_planar_irp6ot* planar_vis;
	//Trajectory description.
	trajectory_description td;

public:
	//Konstruktory.
	ecp_task_pw_scena_irp6ot(configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void task_initialization(void);
	void main_task_algorithm(void);
	//Metods modifing td.
	void set_td_coordinates(double cor0, double cor1, double cor2, double cor3, double cor4, double cor5, double cor6);
	void init_td(POSE_SPECIFICATION ps, int internode_no);
};

#endif
