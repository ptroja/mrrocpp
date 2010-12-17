/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/speaker/ui_r_speaker.h"
#include "ui/src/speaker/ui_ecp_r_speaker.h"
#include "robot/speaker/const_speaker.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace speaker {

//
//
// KLASA UiRobotIrp6ot_m
//
//

int UiRobot::edp_create()

{

	return 1;

}

int UiRobot::edp_create_int()

{

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	//char tmp_string[100];
	//char tmp2_string[100];

	try { // dla bledow robot :: ECP_error

		// dla robota speaker
		if (state.edp.state == 0) {
			state.edp.state = 0;
			state.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += state.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += state.edp.network_resourceman_attach_point;

			// sprawdzeie czy nie jest juz zarejestrowany zarzadca zasobow
			if (((!(state.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0))
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				interface.ui_msg->message("edp_speaker already exists");

			} else if (interface.check_node_existence(state.edp.node_name, "edp_speaker")) {

				state.edp.node_nr = interface.config->return_node_number(state.edp.node_name);

				ui_ecp_robot = new ui::speaker::EcpRobot(&state.edp, *interface.config, *interface.all_ecp_msg);
				state.edp.pid = ui_ecp_robot->get_EDP_pid();

				if (state.edp.pid < 0) {
					state.edp.state = 0;
					fprintf(stderr, "edp spawn failed: %s\n", strerror(errno));
					delete ui_ecp_robot;
				} else { // jesli spawn sie powiodl

					state.edp.state = 1;

					//connect_to_reader();

					//state.edp.state=1;// edp wlaczone reader czeka na start
					state.edp.is_synchronised = true;
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

	return 1;

}

UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::speaker::EDP_SECTION, lib::speaker::ECP_SECTION, lib::speaker::ROBOT_NAME),
			is_wind_speaker_play_open(false), ui_ecp_robot(NULL)
{

}

int UiRobot::reload_configuration()
{

	// jesli speaker ma byc aktywny
	if ((state.is_active = interface.config->value <int> ("is_speaker_active")) == 1) {

		//ui_state.is_any_edp_active = true;
		if (interface.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point
					= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point", state.ecp.section_name);

			state.ecp.pid = -1;
			state.ecp.trigger_fd = lib::invalid_fd;
		}

		switch (state.edp.state)
		{
			case -1:
			case 0:

				state.edp.pid = -1;
				state.edp.reader_fd = lib::invalid_fd;
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

				state.edp.preset_sound_0
						= interface.config->value <std::string> ("preset_sound_0", state.edp.section_name);
				state.edp.preset_sound_1
						= interface.config->value <std::string> ("preset_sound_1", state.edp.section_name);
				state.edp.preset_sound_2
						= interface.config->value <std::string> ("preset_sound_2", state.edp.section_name);

				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}

	} else // jesli  conveyor ma byc nieaktywny
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
	} // end speaker

	return 1;

}

int UiRobot::manage_interface()
{

	switch (state.edp.state)
	{
		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_speaker, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_speaker_edp_unload, ABN_mm_speaker_play, ABN_mm_speaker_preset_sounds, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_speaker, ABN_mm_speaker_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_speaker, NULL);
			//		ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				// ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_speaker_synchronisation, NULL);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_speaker_edp_unload, ABN_mm_speaker_play, ABN_mm_speaker_preset_sounds, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_speaker_edp_load, NULL);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_speaker_play, ABN_mm_speaker_preset_sounds, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_speaker_edp_unload, ABN_mm_speaker_edp_load, NULL);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_speaker_play, ABN_mm_speaker_preset_sounds, NULL);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				/*
				 ApModifyItemState( &robot_menu, AB_ITEM_NORMAL, ABN_mm_speaker_edp_unload, NULL);
				 ApModifyItemState( &robot_menu, AB_ITEM_DIM, ABN_mm_speaker_edp_load, NULL);
				 */
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

	close_wnd_speaker_play(NULL, NULL, NULL);

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
