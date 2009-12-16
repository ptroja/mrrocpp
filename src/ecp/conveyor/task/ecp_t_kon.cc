#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_r_conv.h"
#include "ecp/conveyor/task/ecp_t_kon.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp/common/generator/ecp_g_jarosz.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace task {

// KONSTRUKTORY
kon::kon(lib::configurator &_config) : task(_config)
{
	ecp_m_robot = new robot (*this);

	sr_ecp_msg->message("ECP loaded");
}

void kon::main_task_algorithm(void)
{
	// --------------------------------- old
	// for(;;) {
	// irp6p_msg->message("NOWA SERIA");
	// irp6p_msg->message("Ruch");
	// irp6p_msg->message("Zakocz - nacisnij PULSE ECP trigger");
	// Move (*irp6_postument, slhead, ysg);

	// }
	// ---------------------------------new

	// #####################################################################################
	// TESTOWANIE GENRATOROW o przyrost polozenia/orientacji
	// #####################################################################################

	lib::trajectory_description tdes;

	// -----------------------------------
	// 	tdes.arm_type = lib::XYZ_EULER_ZYZ;
	tdes.arm_type = lib::MOTOR;
	// -----------------------------------

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //
	tdes.interpolation_node_no = 60;
	tdes.internode_step_no =8;
	tdes.value_in_step_no = tdes.internode_step_no - 3;

	/*
        os 0:
        a=40000
        v=10000
        tf=1.5
        s=12500
        ta=0.1667

        	tdes.interpolation_node_no = 90;
        	tdes.internode_step_no =8;
	 */

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

	// --------------------------------------------------------------------------------------------
	/*
        	// Wspolrzedne wewn. JOINT  VER 1

        	tdes.coordinate_delta[0] = 0.0;								// tor jezdny d0
        	tdes.coordinate_delta[1] = 0.3;								// kolumna obrotowa
        	tdes.coordinate_delta[2] = 0.0;								// ramie dolne
        	tdes.coordinate_delta[3] = 0.0;								// ramie gorne
        	tdes.coordinate_delta[4] = 0.0;								// pochylenie kisci
        	tdes.coordinate_delta[5] = 0.0;								// obrot kisci
	 */
	// --------------------------------------------------------------------------------------------

	tdes.coordinate_delta[0] = -150.0;								// tor jezdny d0
	tdes.coordinate_delta[1] = 0.0;  // -150.00;		// kolumna obrotowa (45 stopni)
	// 	tdes.coordinate_delta[1] = -187.50;		// kolumna obrotowa (45 stopni)
	tdes.coordinate_delta[2] = 0.0;								// ramie dolne
	tdes.coordinate_delta[3] = 0.0;								// ramie gorne
	tdes.coordinate_delta[4] = 0.0;								// pochylenie kisci
	tdes.coordinate_delta[5] = 0.0;								// obrot kisci

	// --------------------------------------------------------------------------------------------
	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	/*
        	tdes.coordinate_delta[0] = 0.0;		// przyrost wspolrzednej X
        	tdes.coordinate_delta[1] = 0.0;		// przyrost wspolrzednej Y
        	tdes.coordinate_delta[2] = 0.2;		// przyrost wspolrzednej Z
        	tdes.coordinate_delta[3] = 0.0;		// przyrost wspolrzednej FI
        	tdes.coordinate_delta[4] = 0.0;		// przyrost wspolrzednej TETA
        	tdes.coordinate_delta[5] = 0.0;		// przyrost wspolrzednej PSI
	 */
	// --------------------------------------------------------------------------------------------


	// ####################################################################################################
	// Generator prostoliniowy o zadany przyrost polozenia/orientacji
	// ####################################################################################################
	/*
        	linear_generator lg(tdes);
         	msg->message("Wykonywany jest ruch o zadany przyrost polozenia/orientacji");
        	Move (rnt, NULL, lg);
         	msg->message("Ruch robota zakonczyl sie");
	 */
	// ####################################################################################################
	// Interpolacja funckja liniowa z parabolicznymin odcinkami krzykowliniowymi,
	// czyli trapezoidalny profil predkosci
	// ####################################################################################################

	// double ta[]={0.25,0.25,0.25,0.25,0.25,0.25};
	// double tb[]={0.75, 0.75, 0.75, 0.75, 0.75, 0.75};
	double ta[]={0.25,0.2,0.28,0.17949,0.071795,0.25};
	double tb[]={1-0.25,0.8,0.72,1-0.17949,1-0.071795,0.75};
	common::generator::linear_parabolic trapez(*this, tdes, ta, tb);
	sr_ecp_msg->message("Wykonywany jest ruch o zadany przyrost polozenia/orientacji");
	trapez.Move();

	// postoj
	// 	tdes.coordinate_delta[0] = 0.00;
	// 	linear_parabolic_generator trapez2(tdes, ta, tb);
	// 	Move (rnt, NULL, trapez2);

	// wstecz
	// tdes.coordinate_delta[0] = -150.00;
	// inear_parabolic_generator trapez3(tdes, ta, tb);
	// Move (rnt, NULL, trapez3);
	sr_ecp_msg->message("Ruch robota zakonczyl sie");
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// Informacja dla MP o zakonczeniu zadania uzytkownika
	ecp_termination_notice ();
}

}
} // namespace conveyor

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new conveyor::task::kon(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

