// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - methods
// Funkcje do tworzenia procesow ECP
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_t_rcsc.h"
#include "ecp/common/ecp_g_jarosz.h"

void ecp_gripper_opening (ecp_task& _ecp_task, double gripper_increment, int motion_time)
{

	trajectory_description tdes;

	tdes.arm_type = XYZ_EULER_ZYZ;
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
	ecp_linear_generator lg(_ecp_task, tdes, 1);

	_ecp_task.Move (lg);
};
