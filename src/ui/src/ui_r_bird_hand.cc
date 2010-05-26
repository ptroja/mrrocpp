/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/ui_r_bird_hand.h"
#include "ui/ui_ecp.h"
#include "ui/ui_class.h"

extern Ui ui;

extern ui_state_def ui_state;

//
//
// KLASA UiRobotBirdHand
//
//


UiRobotBirdHand::UiRobotBirdHand() :
	is_wnd_bird_hand_command_and_status_open(false),
			is_wnd_bird_hand_configuration_open(false) {

	state.edp.state = -1; // edp nieaktywne
	state.edp.last_state = -1; // edp nieaktywne
	state.ecp.trigger_fd = -1;
	state.edp.section_name = EDP_BIRD_HAND_SECTION;
	state.ecp.section_name = ECP_BIRD_HAND_SECTION;

}

int UiRobotBirdHand::reload_configuration() {
	// jesli IRP6 on_track ma byc aktywne
	if ((ui.bird_hand.state.is_active = ui.config->value<int> (
			"is_bird_hand_active")) == 1) {
		// ini_con->create_ecp_bird_hand (ini_con->ui->ecp_bird_hand_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active) {
			ui.bird_hand.state.ecp.network_trigger_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"trigger_attach_point",
							ui.bird_hand.state.ecp.section_name);

			ui.bird_hand.state.ecp.pid = -1;
			ui.bird_hand.state.ecp.trigger_fd = -1;
		}

		switch (ui.bird_hand.state.edp.state) {
		case -1:
		case 0:
			// ini_con->create_edp_bird_hand (ini_con->ui->edp_bird_hand_section);

			ui.bird_hand.state.edp.pid = -1;
			ui.bird_hand.state.edp.reader_fd = -1;
			ui.bird_hand.state.edp.state = 0;

			if (ui.config->exists("test_mode",
					ui.bird_hand.state.edp.section_name))
				ui.bird_hand.state.edp.test_mode = ui.config->value<int> (
						"test_mode", ui.bird_hand.state.edp.section_name);
			else
				ui.bird_hand.state.edp.test_mode = 0;

			ui.bird_hand.state.edp.hardware_busy_attach_point
					= ui.config->value<std::string> (
							"hardware_busy_attach_point",
							ui.bird_hand.state.edp.section_name);

			ui.bird_hand.state.edp.network_resourceman_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"resourceman_attach_point",
							ui.bird_hand.state.edp.section_name);

			ui.bird_hand.state.edp.network_reader_attach_point
					= ui.config->return_attach_point_name(
							lib::configurator::CONFIG_SERVER,
							"reader_attach_point",
							ui.bird_hand.state.edp.section_name);

			ui.bird_hand.state.edp.node_name = ui.config->value<std::string> (
					"node_name", ui.bird_hand.state.edp.section_name);
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
		switch (ui.bird_hand.state.edp.state) {
		case -1:
		case 0:
			ui.bird_hand.state.edp.state = -1;
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

