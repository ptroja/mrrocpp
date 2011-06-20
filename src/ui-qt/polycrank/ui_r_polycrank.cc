/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/*W SPKM tez to jest
 #include "ui_r_spkm.h"
 #include "ui_ecp_r_spkm.h"
 #include "wgt_spkm_inc.h"
 */

#include "wgt_polycrank_int.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common.h"
#include "ui_r_polycrank.h"
#include "robot/polycrank/const_polycrank.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"
#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"


namespace mrrocpp {
namespace ui {
namespace polycrank {

//
//
// KLASA UiRobot
//
//

int UiRobot::ui_get_edp_pid()
{
	return ui_ecp_robot->ecp->get_EDP_pid();
}

void UiRobot::ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	ui_ecp_robot->get_controller_state(robot_controller_initial_state_l);

}

UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS), ui_ecp_robot(NULL)
{

	wgt_int = new wgt_polycrank_int(interface, *this, interface.get_main_window());
	wndbase_m[WGT_POLYCRANK_INT] = wgt_int->dwgt;

	//wgt_inc = new wgt_spkm_inc(interface, *this, interface.mw);
	//wndbase_m[WGT_SPKM_INC] = wgt_inc->dwgt;


	/* TR
	 wnd_int = new WndInt(interface, *this);
	 wndbase_m[wnd_int->window_name] = wnd_int;
	 wnd_external = new WndExternal(interface, *this);
	 wndbase_m[wnd_external->window_name] = wnd_external;
	 */

}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new ui::common::EcpRobot(*this);
	return 1;
}

int UiRobot::synchronise()
{
	eb.command(boost::bind(&ui::polycrank::UiRobot::synchronise_int, &(*this)));
	//Polycrank will be always in synchronise poses
	return 1;

}

int UiRobot::synchronise_int()
{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota polycrank


		if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp polycrank niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();

	return 1;

}

int UiRobot::manage_interface()
{
	MainWindow *mw = interface.get_main_window();


	switch (state.edp.state)
	{
		case -1:
			mw->enable_menu_item(false, 1, robot_menu);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank, NULL);
			 */
			break;
		case 0:
			mw->enable_menu_item(false, 1, EDP_Unload);
			mw->enable_menu_item(true, 1, EDP_Load);
			mw->enable_menu_item(true, 1, robot_menu);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_unload, ABN_mm_polycrank_internal, NULL);
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank, ABN_mm_polycrank_edp_load, NULL);
			 */
			break;
		case 1:
		case 2:
			mw->enable_menu_item(true, 1, robot_menu);
			/* TR
			 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank, NULL);
			 */
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				//??
				/* TR
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				 ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_internal, NULL);
				 */
				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						mw->enable_menu_item(true, 1, EDP_Unload);
						mw->enable_menu_item(false, 1,EDP_Load);
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_edp_unload, ABN_mm_polycrank_internal, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, NULL);
						 */
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						mw->enable_menu_item(false, 2, EDP_Unload, EDP_Load);
						block_ecp_trigger();
						/* TR
						 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_internal, NULL);
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, ABN_mm_polycrank_edp_unload, NULL);
						 */
						break;
					case common::UI_MP_TASK_RUNNING:
						unblock_ecp_trigger();
						break;
					case common::UI_MP_TASK_PAUSED:
						block_ecp_trigger();
						/* TR
						 //ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);// modyfikacja menu - ruchy reczne zakazane
						 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_internal, NULL);// modyfikacja menu - ruchy reczne zakazane
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
				 ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_edp_unload, NULL);
				 ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, NULL);
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
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	connect(actionpolycrank_Move_Joints,	SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Move_Joints_triggered(mrrocpp::ui::common::UiRobot*)),	Qt::AutoCompatConnection);
}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();

	actionpolycrank_Move_Joints = new Ui::MenuBarAction(QString("Move &Joints"), this, menuBar);

	robot_menu->addSeparator();
	robot_menu->addAction(actionpolycrank_Move_Joints);

	menuBar->menuRobot->addAction(robot_menu->menuAction());
	robot_menu->setTitle(QApplication::translate("MainWindow", "Polyc&rank", 0, QApplication::UnicodeUTF8));

	make_connections();
}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

void UiRobot::null_ui_ecp_robot()
{
	ui_ecp_robot = NULL;

}

}
} //namespace ui
} //namespace mrrocpp
