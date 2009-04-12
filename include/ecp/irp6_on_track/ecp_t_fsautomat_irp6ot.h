// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Realizacja automatu skonczonego - ECP dla IRP6_ON_TRACK
//
// Ostatnia modyfikacja: 	2008
// autor:						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_T_FSAUTOMAT_IRP6OT_H)
#define _ECP_T_FSAUTOMAT_IRP6OT_H

#include <map>

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_generator_t.h"

#include "mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

class ecp_task_fsautomat_irp6ot: public common::ecp_task
{
	protected:
		// generatory
		common::ecp_smooth_generator* sg;
		common::ecp_tool_change_generator* tcg;
		common::ecp_generator_t* gt;
		common::ecp_tff_nose_run_generator* nrg;
		common::ecp_tff_rubik_grab_generator* rgg;
		common::ecp_tff_gripper_approach_generator* gag;
		common::ecp_tff_rubik_face_rotate_generator* rfrg;
		common::ecp_teach_in_generator* tig;
		common::bias_edp_force_generator* befg;
		common::weight_meassure_generator* wmg;
		//podzadania
		common::ecp_sub_task_gripper_opening* go_st;


		std::map<char*, mp::common::Trajectory, str_cmp>* trjMap;

	public:
		// KONSTRUKTORY
		ecp_task_fsautomat_irp6ot(configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
//		void grip(double gripper_increment, int motion_time);

};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif// -------------------------------------------------------------------------
