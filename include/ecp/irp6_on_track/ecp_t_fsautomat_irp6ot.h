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
namespace task {

class ecp_task_fsautomat_irp6ot: public common::task::ecp_task
{
	protected:
		// generatory
		common::generator::smooth* sg;
		common::generator::tool_change* tcg;
		common::generator::transparent* gt;
		common::generator::tff_nose_run* nrg;
		common::generator::tff_rubik_grab* rgg;
		common::generator::tff_gripper_approach* gag;
		common::generator::tff_rubik_face_rotate* rfrg;
		common::ecp_teach_in_generator* tig;
		common::generator::bias_edp_force* befg;
		common::generator::weight_meassure* wmg;
		//podzadania
		common::task::ecp_sub_task_gripper_opening* go_st;


		std::map<char*, mp::common::Trajectory, str_cmp>* trjMap;

	public:
		// KONSTRUKTORY
		ecp_task_fsautomat_irp6ot(configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
//		void grip(double gripper_increment, int motion_time);

};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif// -------------------------------------------------------------------------
