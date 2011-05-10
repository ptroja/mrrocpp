/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_shead.h"
#include "ui_ecp_r_shead.h"
#include "robot/shead/const_shead.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

namespace mrrocpp {
namespace ui {
namespace shead {

//
//
// KLASA UiRobotIrp6ot_m
//
//

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
	ui_ecp_robot = new ui::shead::EcpRobot(interface);
	return 1;
}

int UiRobot::synchronise()

{

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
			common::UiRobot(_interface, lib::shead::EDP_SECTION, lib::shead::ECP_SECTION, lib::shead::ROBOT_NAME, lib::shead::NUM_OF_SERVOS),
			ui_ecp_robot(NULL)
{

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();
	Ui::MainWindow *ui = mw->get_ui();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, ui->menuShead);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead, NULL);
			 */
			break;
		case 0:
			mw->enable_menu_item(false, 1, ui->actionshead_EDP_Unload);
			mw->enable_menu_item(true, 1, ui->menuShead);
			mw->enable_menu_item(true, 1, ui->actionshead_EDP_Load);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead_edp_unload,

			 NULL);
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_shead, ABN_mm_shead_edp_load, NULL);
			 */
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, ui->menuShead);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_shead, NULL);
			 */
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(true, 1, ui->menuall_Preset_Positions);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
				 */
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 1, ui->actionshead_EDP_Unload);
						mw->enable_menu_item(false, 1, ui->actionshead_EDP_Load);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_shead_edp_unload, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead_edp_load, NULL);
						 */
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(false, 2, ui->actionshead_EDP_Unload, ui->actionshead_EDP_Load);
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

						 NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead_edp_load, ABN_mm_shead_edp_unload, NULL);
						 */
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						 NULL);
						 */
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				mw->enable_menu_item(true, 1, ui->actionshead_EDP_Unload);
				mw->enable_menu_item(false, 1, ui->actionshead_EDP_Load);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_shead_edp_unload, NULL);
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_shead_edp_load, NULL);
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

