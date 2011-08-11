/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_sbench.h"
#include "ui_ecp_r_sbench.h"
#include "robot/sbench/const_sbench.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace sbench {

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

int UiRobot::synchronise()

{

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::sbench::ROBOT_NAME, lib::sbench::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();

	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, robot_menu);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_sbench, NULL);
			 */
			break;
		case 0:
			mw->enable_menu_item(false, 1, EDP_Unload);
			mw->enable_menu_item(true, 1, robot_menu);
			mw->enable_menu_item(true, 1, EDP_Load);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_sbench_edp_unload,

			 NULL);
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_sbench, ABN_mm_sbench_edp_load, NULL);
			 */
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, robot_menu);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_sbench, NULL);
			 */
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				mw->enable_menu_item(true, 1, mw->getMenuBar()->menuall_Preset_Positions);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);
				 */
				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 1, EDP_Unload);
						mw->enable_menu_item(false, 1, EDP_Load);
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_sbench_edp_unload, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_sbench_edp_load, NULL);
						 */
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(false, 2, EDP_Unload, EDP_Load);
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

						 NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_sbench_edp_load, ABN_mm_sbench_edp_unload, NULL);
						 */
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						block_ecp_trigger();
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
				mw->enable_menu_item(true, 1, EDP_Unload);
				mw->enable_menu_item(false, 1, EDP_Load);
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_sbench_edp_unload, NULL);
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_sbench_edp_load, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
				 */
			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::make_connections()
{

}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	//	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	robot_menu->setTitle(QApplication::translate("MainWindow", "S&head", 0, QApplication::UnicodeUTF8));
	make_connections(); //zostawione na przyszłośc (narazie nic nie robi)
	robot_menu->setTitle(QApplication::translate("MainWindow", "S&bench", 0, QApplication::UnicodeUTF8));

}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;

}

void UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new EcpRobot(*this);
	//	return 1;
}

}
} //namespace ui
} //namespace mrrocpp

