/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_ecp_r_bird_hand.h"
#include "ui_r_bird_hand.h"

#include "wgt_bird_hand_command.h"
/* #include "ui/src/bird_hand/wnd_bird_hand_configuration.h"
 */
#include "robot/bird_hand/const_bird_hand.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

//
// KLASA UiRobotBirdHand
//
//

namespace mrrocpp {
namespace ui {
namespace bird_hand {

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->the_robot->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

void UiRobot::edp_create()
{
	if (state.edp.state == 0) {
		create_thread();

		eb.command(boost::bind(&ui::bird_hand::UiRobot::edp_create_int, &(*this)));
	}
}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::bird_hand::EcpRobot(interface);
	return 1;
}

int UiRobot::synchronise()

{

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
			common::UiRobot(_interface, lib::bird_hand::EDP_SECTION, lib::bird_hand::ECP_SECTION, lib::bird_hand::ROBOT_NAME, lib::bird_hand::NUM_OF_SERVOS),
			ui_ecp_robot(NULL)
{

	wgt_command_and_status = new wgt_bird_hand_command(interface, *this, interface.get_main_window());
	wndbase_m[WGT_BIRD_HAND_COMMAND] = wgt_command_and_status->dwgt;
	// wndbase_m[wnd_command_and_status->window_name] = wnd_command_and_status;
	/* wnd_configuration = new WndConfiguration(interface, *this);
	 wndbase_m[wnd_configuration->window_name] = wnd_configuration;
	 */

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, ui->menuBirdhand);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand, NULL);
			 */
			break;
		case 0:
			mw->enable_menu_item(false, 3, ui->actionbirdhand_EDP_Unload, ui->actionbirdhand_Command, ui->actionbirdhand_Configuration);
			mw->enable_menu_item(true, 1, ui->menuBirdhand);
			mw->enable_menu_item(true, 1, ui->actionbirdhand_EDP_Load);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_unload, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand, ABN_mm_bird_hand_edp_load, NULL);
			 */

			break;
		case 1:
		case 2:
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand, NULL);
			 */
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				//mw->enable_menu_item(false, 1, ui->menuRobot); //??
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
				 */
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 3, ui->actionbirdhand_EDP_Unload, ui->actionbirdhand_Command, ui->actionbirdhand_Configuration);
						mw->enable_menu_item(false, 1, ui->actionbirdhand_EDP_Load);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand_edp_unload, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_load, NULL);
						 */
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(true, 2, ui->actionbirdhand_Command, ui->actionbirdhand_Configuration);
						mw->enable_menu_item(false, 2, ui->actionbirdhand_EDP_Unload, ui->actionbirdhand_EDP_Load);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_edp_unload, NULL);
						 */
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						mw->enable_menu_item(false, 2, ui->actionbirdhand_Command, ui->actionbirdhand_Configuration);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						 ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
						 */
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, ui->actionbirdhand_EDP_Unload);
				mw->enable_menu_item(false, 3, ui->actionbirdhand_EDP_Load, ui->actionbirdhand_Command, ui->actionbirdhand_Configuration);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand_edp_unload, NULL);
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
				 */
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

}
} //namespace ui
} //namespace mrrocpp
