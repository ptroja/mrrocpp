/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_spkm.h"
#include "ui_ecp_r_spkm.h"
#include "wgt_spkm_inc.h"
#include "wgt_spkm_int.h"
#include "wgt_spkm_ext.h"

#include "robot/spkm/const_spkm.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

//
//
// KLASA UiRobotIrp6ot_m
//
//


UiRobot::UiRobot(common::Interface& _interface) :
			common::UiRobot(_interface, lib::spkm::EDP_SECTION, lib::spkm::ECP_SECTION, lib::spkm::ROBOT_NAME, lib::spkm::NUM_OF_SERVOS),
			ui_ecp_robot(NULL)
{

	wgt_inc = new wgt_spkm_inc(interface, *this, interface.get_main_window());
	wndbase_m[WGT_SPKM_INC] = wgt_inc->dwgt;

	wgt_int = new wgt_spkm_int(interface, *this, interface.get_main_window());
	wndbase_m[WGT_SPKM_INT] = wgt_int->dwgt;

	wgt_ext = new wgt_spkm_ext(interface, *this, interface.get_main_window());
	wndbase_m[WGT_SPKM_EXT] = wgt_ext->dwgt;

}

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->the_robot->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::spkm::EcpRobot(interface);
	return 1;
}

int UiRobot::edp_create_int_extra_operations()
{
	wgt_inc->synchro_depended_init();
	return 1;
}

int UiRobot::synchronise()

{

	eb.command(boost::bind(&ui::spkm::UiRobot::synchronise_int, &(*this)));

	return 1;

}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota spkm

		if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->the_robot->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->the_robot->is_synchronised();
		} else {
			// 	printf("edp spkm niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	interface.manage_interface();
	wgt_inc->synchro_depended_init();

	return 1;

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, ui->menuSpkm);
			break;
		case 0:
			mw->enable_menu_item(false, 1, ui->actionspkm_EDP_Unload);
			mw->enable_menu_item(false, 1, ui->actionspkm_Clear_Fault);
			mw->enable_menu_item(false, 3, ui->menuspkm_Pre_synchro_moves, ui->menuspkm_Preset_positions, ui->menuspkm_Post_synchro_moves);
			mw->enable_menu_item(true, 1, ui->menuSpkm);
			mw->enable_menu_item(true, 1, ui->actionspkm_EDP_Load);

			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, ui->menuSpkm);
			mw->enable_menu_item(true, 1, ui->actionspkm_Clear_Fault);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(false, 1, ui->menuspkm_Pre_synchro_moves);
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 2, ui->menuspkm_Preset_positions, ui->menuspkm_Post_synchro_moves);
						mw->enable_menu_item(true, 1, ui->actionspkm_EDP_Unload); //???
						mw->enable_menu_item(false, 1, ui->actionspkm_EDP_Load);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 2, ui->menuspkm_Preset_positions, ui->menuspkm_Post_synchro_moves);//???
						mw->enable_menu_item(false, 2, ui->actionspkm_EDP_Load, ui->actionspkm_EDP_Unload);

						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 2, ui->menuspkm_Preset_positions, ui->menuspkm_Post_synchro_moves);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, ui->actionspkm_EDP_Unload);
				mw->enable_menu_item(true, 1, ui->menuspkm_Pre_synchro_moves);
				mw->enable_menu_item(false, 1, ui->actionspkm_EDP_Load);
			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

int UiRobot::move_to_synchro_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = 0.0;
	}
	eb.command(boost::bind(&ui::spkm::UiRobot::execute_motor_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_front_position()
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.front_position[i];
	}
	eb.command(boost::bind(&ui::spkm::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{

	for (int i = 0; i < number_of_servos; i++) {
		desired_pos[i] = state.edp.preset_position[variant][i];
	}
	eb.command(boost::bind(&ui::spkm::UiRobot::execute_joint_motion, &(*this)));

	return 1;
}

int UiRobot::execute_motor_motion()
{
	try {

		ui_ecp_robot->move_motors(desired_pos, lib::epos::NON_SYNC_TRAPEZOIDAL);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int UiRobot::execute_joint_motion()
{
	try {

		ui_ecp_robot->move_joints(desired_pos, lib::epos::NON_SYNC_TRAPEZOIDAL);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int UiRobot::execute_clear_fault()
{
	try {

		ui_ecp_robot->clear_fault();

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int UiRobot::execute_stop_motor()
{
	try {

		ui_ecp_robot->stop_motors();

	} // end try
	CATCH_SECTION_UI

	return 1;
}

}
} //namespace ui
} //namespace mrrocpp

