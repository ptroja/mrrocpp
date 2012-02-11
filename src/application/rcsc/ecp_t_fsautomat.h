// -------------------------------------------------------------------------
//                            task/ecp_t_fsautomat.h dla QNX6
// Definicje struktur danych i metod dla procesow ECP
//
// Ostatnia modyfikacja:	sierpien 2008
// Autor:						Marek Kisiel
// -------------------------------------------------------------------------

#if !defined(_ECP_TASK_FSAUTOMAT_H)
#define _ECP_TASK_FSAUTOMAT_H

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>

#include "base/ecp/ecp_task.h"
#include "generator/ecp/transparent/ecp_g_transparent.h"
#include "generator/ecp/weight_measure/ecp_g_weight_measure.h"
#include "generator/ecp/tff_nose_run/ecp_g_tff_nose_run.h"
#include "generator/ecp/teach_in/ecp_g_teach_in.h"
#include "generator/ecp/tff_rubik_face_rotate/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/tff_gripper_approach/ecp_g_tff_gripper_approach.h"
#include "base/lib/trajectory_pose/trajectory_pose.h"
#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "base/lib/datastr.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//void ecp_gripper_opening (task& ecp_object, double gripper_increment, int motion_time);

class fsautomat : public task
{
protected:
	// generatory
	common::generator::newsmooth* sg; //changed askubis
	common::generator::transparent* gt;
	common::generator::tff_nose_run* nrg;

	common::generator::tff_gripper_approach* gag;
	common::generator::tff_rubik_face_rotate* rfrg;
	common::generator::teach_in* tig;
	common::generator::bias_edp_force* befg;
	common::generator::weight_measure* wmg;

	bang_trajectories_map trjMap;
	int axes_num;

	void load_trajectory_from_xml(std::pair <std::vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose *>,
			lib::MOTION_TYPE> pair_trj_motion);
	void load_trajectory_from_xml(const char* fileName, const char* nodeName);
	void load_file_with_path(const char* file_name);
	void set_pose_from_xml(xmlNode *stateNode, bool &first_time);

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
