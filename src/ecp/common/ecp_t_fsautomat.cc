// -------------------------------------------------------------------------
//                            ecp_t_fsautomat.cc
//
// Funkcje do obslugi chwytaka, zacytowane z ecp_t_rcsc.cc
//
// Ostatnia modyfikacja: 2008
// -------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_t_fsautomat.h"
#include "ecp/common/ecp_g_jarosz.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

void ecp_gripper_opening (common::task::base& _ecp_task, double gripper_increment, int motion_time)
{

	lib::trajectory_description tdes;

	tdes.arm_type = lib::XYZ_EULER_ZYZ;
	tdes.interpolation_node_no = 1;
	tdes.internode_step_no = motion_time;
	tdes.value_in_step_no = tdes.internode_step_no - 2;
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	tdes.coordinate_delta[0] = 0.0; // przyrost wspolrzednej X
	tdes.coordinate_delta[1] = 0.0;// przyrost wspolrzednej Y
	tdes.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z
	tdes.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	tdes.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	tdes.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI
	//	tdes.coordinate_delta[6] = 0.0;   // przyrost wspolrzednej PSI
	tdes.coordinate_delta[6] = gripper_increment;   // przyrost wspolrzednej PSI

	// Generator trajektorii prostoliniowej
	linear lg(_ecp_task, tdes, 1);

	_ecp_task.Move (lg);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
