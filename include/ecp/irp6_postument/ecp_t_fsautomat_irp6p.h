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
namespace common {

class ecp_task_fsautomat_irp6p: public ecp_task
{
	protected:
		// generatory
		ecp_smooth_generator* sg;
		ecp_tool_change_generator* tcg;
		ecp_generator_t* gt;
		ecp_tff_nose_run_generator* nrg;
		ecp_tff_rubik_grab_generator* rgg;
		ecp_tff_gripper_approach_generator* gag;
		ecp_tff_rubik_face_rotate_generator* rfrg;
		ecp_teach_in_generator* tig;
		bias_edp_force_generator* befg;
		weight_meassure_generator* wmg;
		//podzadania
		ecp_sub_task_gripper_opening* go_st;

		std::map<char*, mp::common::Trajectory, str_cmp>* trjMap;

	public:
		// KONSTRUKTORY
		ecp_task_fsautomat_irp6p(configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
//		bool loadTrajectories(char * fileName);

};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
