/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_conveyor.h"
#include "ui_ecp_r_conveyor.h"
#include "robot/conveyor/const_conveyor.h"
#include "../base/interface.h"

#include "../base/mainwindow.h"
#include "ui_mainwindow.h"

#include "../base/wgt_single_motor_move.h"

#include "../base/signal_dispatcher.h"

#include "../base/menu_bar.h"
#include "../base/menu_bar_action.h"
#include "../base/mp.h"

namespace mrrocpp {
namespace ui {
namespace conveyor {
const std::string WGT_CONVEYOR_MOVE = "WGT_CONVEYOR_MOVE";

void UiRobot::create_ui_ecp_robot()
{
	common::UiRobot::ui_ecp_robot = ui_ecp_robot = new ui::conveyor::EcpRobot(*this);
}

void UiRobot::edp_create_int_extra_operations()
{
	wgts[WGT_CONVEYOR_MOVE]->synchro_depended_init();
}

void UiRobot::synchronise()
{
	eb.command(boost::bind(&ui::conveyor::UiRobot::synchronise_int, &(*this)));
}

int UiRobot::synchronise_int()

{

	interface.set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6_on_track

		if ((is_edp_loaded()) && (state.edp.is_synchronised == false)) {
			ui_ecp_robot->ecp->synchronise();
			state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
		} else {
			// 	printf("edp irp6_on_track niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_IN_ROBOT

	// modyfikacje menu
	interface.manage_interface();
	wgts[WGT_CONVEYOR_MOVE]->synchro_depended_init();
	wgts[WGT_CONVEYOR_MOVE]->init_and_copy();
	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
		single_motor::UiRobot(_interface, lib::conveyor::ROBOT_NAME, lib::conveyor::NUM_OF_SERVOS)
{
	add_wgt <wgt_single_motor_move>(WGT_CONVEYOR_MOVE, "Conveyor moves");

}

void UiRobot::manage_interface()
{

	single_motor::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			actionconveyor_Move->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
					case common::UI_MP_WAITING_FOR_START_PULSE:
						actionconveyor_Move->setEnabled(true);

						break;
					case common::UI_MP_TASK_RUNNING:
						break;
					case common::UI_MP_TASK_PAUSED:
						actionconveyor_Move->setEnabled(false);

						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				actionconveyor_Move->setEnabled(true);

			}
			break;
		default:
			break;
	}

}

void UiRobot::setup_menubar()
{
	single_motor::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	actionconveyor_Move = new Ui::MenuBarAction(QString("&Move"), wgts[WGT_CONVEYOR_MOVE], signalDispatcher, menuBar);
	robot_menu->addAction(actionconveyor_Move);

	robot_menu->setTitle(QApplication::translate("MainWindow", "&Conveyor", 0, QApplication::UnicodeUTF8));
}

}
}
}
