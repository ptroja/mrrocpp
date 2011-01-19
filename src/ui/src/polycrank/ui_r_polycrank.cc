/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/ui_ecp_r_tfg_and_conv.h"
#include "ui/src/polycrank/ui_r_polycrank.h"
#include "robot/polycrank/const_polycrank.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace polycrank {

//
//
// KLASA UiRobot
//
//

void UiRobot::edp_create()
{
	if (state.edp.state == 0) {
		create_thread();

		eb.command(boost::bind(&ui::polycrank::UiRobot::edp_create_int, &(*this)));
	}
}

int UiRobot::edp_create_int()
{

	interface.set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota polycrank
		if (state.edp.state == 0) {

			state.edp.state = 0;
			state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += state.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(state.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0))
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "edp_polycrank already exists");
			} else if (interface.check_node_existence(state.edp.node_name, "edp_polycrank")) {

				state.edp.node_nr = interface.config->return_node_number(state.edp.node_name);

				{
					boost::unique_lock <boost::mutex> lock(interface.process_creation_mtx);

					ui_ecp_robot = new ui::tfg_and_conv::EcpRobot(interface, lib::polycrank::ROBOT_NAME);

				}

				state.edp.pid = ui_ecp_robot->ecp->get_EDP_pid();

				if (state.edp.pid < 0) {

					state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					state.edp.state = 1;

					connect_to_reader();

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui_ecp_robot->get_controller_state(robot_controller_initial_state_tmp);

					//state.edp.state = 1; // edp wlaczone reader czeka na start

					state.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	interface.manage_interface();

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
	CATCH_SECTION_UI

	// modyfikacje menu
	interface.manage_interface();

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
			common::UiRobot(_interface, lib::polycrank::EDP_SECTION, lib::polycrank::ECP_SECTION, lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS, "is_polycrank_active"),
			is_wind_polycrank_int_open(false), ui_ecp_robot(NULL)
{
}

int UiRobot::manage_interface()
{
	switch (state.edp.state)
	{
		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_unload, ABN_mm_polycrank_internal, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank, ABN_mm_polycrank_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank, NULL);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_internal, NULL);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_edp_unload, ABN_mm_polycrank_internal, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, NULL);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_internal, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, ABN_mm_polycrank_edp_unload, NULL);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						//ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);// modyfikacja menu - ruchy reczne zakazane
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_internal, NULL);// modyfikacja menu - ruchy reczne zakazane
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_edp_unload, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
			}
			break;
		default:
			break;
	}

	return 1;
}

void UiRobot::close_all_windows()
{
	int pt_res = PtEnter(0);

	close_wnd_polycrank_int(NULL, NULL, NULL);

	if (pt_res >= 0) {
		PtLeave(0);
	}
}

void UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

}
} //namespace ui
} //namespace mrrocpp
