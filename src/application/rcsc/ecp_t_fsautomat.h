// -------------------------------------------------------------------------
//                            task/ecp_t_fsautomat.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
//
// Ostatnia modyfikacja:	sierpien 2008
// Autor:						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_TASK_FSAUTOMAT_H)
#define _ECP_TASK_FSAUTOMAT_H

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_st_go.h"
#include "base/ecp/ecp_g_transparent.h"
#include "generator/ecp/force/ecp_g_weight_measure.h"
#include "generator/ecp/force/ecp_g_tff_nose_run.h"
#include "generator/ecp/force/ecp_g_tff_rubik_grab.h"
#include "generator/ecp/force/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "base/ecp_mp/Trajectory.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//void ecp_gripper_opening (task& ecp_object, double gripper_increment, int motion_time);

class fsautomat : public task
{
protected:
	// generatory
	//common::generator::smooth* sg;
	common::generator::transparent* gt;
	common::generator::tff_nose_run* nrg;
	common::generator::tff_rubik_grab* rgg;
	common::generator::tff_gripper_approach* gag;
	common::generator::tff_rubik_face_rotate* rfrg;
	common::generator::teach_in* tig;
	common::generator::bias_edp_force* befg;
	common::generator::weight_measure* wmg;
	//podzadania
	common::sub_task::gripper_opening* go_st;

	trajectories_t * trjMap;

public:
	// KONSTRUKTORY
	fsautomat(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
