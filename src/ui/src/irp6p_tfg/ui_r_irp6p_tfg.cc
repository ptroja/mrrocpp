/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/irp6p_tfg/ui_r_irp6p_tfg.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace irp6p_tfg {

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::irp6p_tfg::EDP_SECTION, lib::irp6p_tfg::ECP_SECTION),
			is_wind_irp6p_tfg_moves_open(false), is_wind_irp6p_tfg_servo_algorithm_open(false), ui_ecp_robot(NULL)
{

}

int UiRobot::reload_configuration()
{
	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = interface.config->value <int> ("is_irp6p_tfg_active")) == 1) {
		// ini_con->create_ecp_irp6p_tfg (ini_con->ui->ecp_irp6p_tfg_section);
		//ui_state.is_any_edp_active = true;
		if (interface.is_mp_and_ecps_active) {
			state.ecp.network_trigger_attach_point
					= interface.config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point", state.ecp.section_name);

			state.ecp.pid = -1;
			state.ecp.trigger_fd = -1;
		}

		switch (state.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6p_tfg (ini_con->ui->edp_irp6p_tfg_section);

				state.edp.pid = -1;
				state.edp.reader_fd = common::edp_state_def::invalid_reader_fd;
				state.edp.state = 0;

				for (int i = 0; i < 3; i++) {
					char tmp_string[50];
					sprintf(tmp_string, "preset_position_%d", i);

					if (interface.config->exists(tmp_string, state.edp.section_name)) {
						char* tmp, *tmp1;
						tmp1 = tmp
								= strdup(interface.config->value <std::string> (tmp_string, state.edp.section_name).c_str());
						char* toDel = tmp;
						for (int j = 0; j < lib::irp6p_tfg::NUM_OF_SERVOS; j++) {

							state.edp.preset_position[i][j] = strtod(tmp1, &tmp1);

						}
						free(toDel);
					} else {
						for (int j = 0; j < lib::irp6p_tfg::NUM_OF_SERVOS; j++) {

							state.edp.preset_position[i][j] = 0.074;

						}
					}
				}

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
	} // end irp6p_tfg

	return 1;
}

int UiRobot::manage_interface()
{
	switch (state.edp.state)
	{
		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_unload, ABN_mm_irp6p_tfg_synchronisation, ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg, ABN_mm_irp6p_tfg_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg, NULL);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_synchronisation, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg_edp_unload, ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_load, NULL);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_load, ABN_mm_irp6p_tfg_edp_unload, NULL);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg_edp_unload, ABN_mm_irp6p_tfg_synchronisation, ABN_mm_irp6p_tfg_move, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_load, NULL);
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

	close_wind_irp6p_tfg_moves(NULL, NULL, NULL);
	close_wnd_irp6p_tfg_servo_algorithm(NULL, NULL, NULL);

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

