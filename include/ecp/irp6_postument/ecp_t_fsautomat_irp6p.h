// -------------------------------------------------------------------------
//                            ecp.h dla QNX6
// Realizacja automatu skonczonego - ECP dla IRP6_POSTUMENT
// Ostatnia modyfikacja: 	2008
// autor: 						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_T_FSAUTOMAT_IRP6P_H)
#define _ECP_T_FSAUTOMAT_IRP6P_H

#include <map>

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_generator_t.h"

#include "mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

class ecp_task_fsautomat_irp6p: public common::task::ecp_task
{
	protected:
		// generatory
		common::generator::ecp_smooth_generator* sg;
		common::generator::ecp_tool_change_generator* tcg;
		common::generator::ecp_generator_t* gt;
		common::generator::ecp_tff_nose_run_generator* nrg;
		common::generator::ecp_tff_rubik_grab_generator* rgg;
		common::generator::ecp_tff_gripper_approach_generator* gag;
		common::generator::ecp_tff_rubik_face_rotate_generator* rfrg;
		common::ecp_teach_in_generator* tig;
		common::generator::bias_edp_force_generator* befg;
		common::generator::weight_meassure_generator* wmg;
		//podzadania
		common::task::ecp_sub_task_gripper_opening* go_st;

		std::map<char*, mp::common::Trajectory, str_cmp>* trjMap;

	public:
		// KONSTRUKTORY
		ecp_task_fsautomat_irp6p(configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
//		bool loadTrajectories(char * fileName);

};

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
