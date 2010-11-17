/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "ui/src/bird_hand/ui_r_bird_hand.h"
#include "ui/src/bird_hand/wnd_bird_hand_command_and_status.h"
#include "ui/src/bird_hand/wnd_bird_hand_configuration.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

//
//
// KLASA UiRobotBirdHand
//
//

namespace mrrocpp {
namespace ui {
namespace bird_hand {

UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::bird_hand::EDP_SECTION, lib::bird_hand::ECP_SECTION), ui_ecp_robot(NULL)
{
	wnd_command_and_status = new WndCommandAndStatus(interface, *this);
	wnd_configuration = new WndConfiguration(interface, *this);

}

int UiRobot::reload_configuration()
{
	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = interface.config->value <int> ("is_bird_hand_active")) == 1) {
		// ini_con->create_ecp_bird_hand (ini_con->ui->ecp_bird_hand_section);
		//ui_state.is_any_edp_active = true;
		if (interface.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point
					= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point", state.ecp.section_name);

			state.ecp.pid = -1;
			state.ecp.trigger_fd = common::invalid_fd;
		}

		switch (state.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_bird_hand (ini_con->ui->edp_bird_hand_section);

				state.edp.pid = -1;
				state.edp.reader_fd = common::invalid_fd;
				state.edp.state = 0;

				if (interface.config->exists(lib::ROBOT_TEST_MODE, state.edp.section_name))
					state.edp.test_mode = interface.config->value <int> (lib::ROBOT_TEST_MODE, state.edp.section_name);
				else
					state.edp.test_mode = 0;

				state.edp.hardware_busy_attach_point
						= interface.config->value <std::string> ("hardware_busy_attach_point", state.edp.section_name);

				state.edp.network_resourceman_attach_point
						= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", state.edp.section_name);

				state.edp.network_reader_attach_point
						= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point", state.edp.section_name);

				state.edp.node_name = interface.config->value <std::string> ("node_name", state.edp.section_name);
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}

	} else // jesli  irp6 on_track ma byc nieaktywne
	{
		switch (state.edp.state)
		{
			case -1:
			case 0:
				state.edp.state = -1;
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}
	} // end bird_hand

	return 1;
}

int UiRobot::manage_interface()
{
	switch (state.edp.state)
	{
		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_unload, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand, ABN_mm_bird_hand_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand, NULL);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand_edp_unload, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_load, NULL);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_edp_unload, NULL);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_bird_hand_edp_unload, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_bird_hand_edp_load, ABN_mm_bird_hand_command, ABN_mm_bird_hand_configuration, NULL);
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

	close_wnd_bird_hand_command_and_status(NULL, NULL, NULL);
	close_wnd_bird_hand_configuration(NULL, NULL, NULL);

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
