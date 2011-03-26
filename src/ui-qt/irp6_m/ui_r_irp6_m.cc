/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "../irp6_m/wgt_irp6_m_motors.h"
#include "../irp6_m/wgt_irp6_m_joints.h"

#include "ui_r_irp6_m.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"

#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

namespace mrrocpp {
namespace ui {
namespace irp6_m {

//
//
// KLASA UiRobot
//
//


int UiRobot::execute_motor_motion()
{
	try {

		ui_ecp_robot->move_motors(desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int UiRobot::execute_joint_motion()
{
	try {

		ui_ecp_robot->move_joints(desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_on_track

		if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_on_track niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	interface.manage_interface();
	wgt_motors->synchro_depended_init();
	wgt_motors->init_and_copy();

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface, const std::string & edp_section_name, const std::string & ecp_section_name, lib::robot_name_t _robot_name, int _number_of_servos, const std::string & _activation_string) :
			common::UiRobot(_interface, edp_section_name, ecp_section_name, _robot_name, _number_of_servos, _activation_string),
			ui_ecp_robot(NULL)

{

}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

}
} //namespace ui
} //namespace mrrocpp
