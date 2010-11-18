/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/irp6p_m/ui_r_irp6p_m.h"
#include "ui/src/ui_ecp_r_irp6_common.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace irp6p_m {

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::irp6p_m::EDP_SECTION, lib::irp6p_m::ECP_SECTION), is_wind_irp6p_int_open(false),
			is_wind_irp6p_inc_open(false), is_wind_irp6p_xyz_euler_zyz_open(false),
			is_wind_irp6p_xyz_angle_axis_open(false), is_wind_irp6p_xyz_aa_relative_open(false),
			is_wind_irp6p_xyz_angle_axis_ts_open(false), is_wind_irp6p_xyz_euler_zyz_ts_open(false),
			is_wind_irp6p_kinematic_open(false), is_wind_irp6p_servo_algorithm_open(false), ui_ecp_robot(NULL)
{

}

int UiRobot::reload_configuration()
{

	// jesli IRP6 postument ma byc aktywne
	if ((state.is_active = interface.config->value <int> ("is_irp6p_m_active")) == 1) {
		// ini_con->create_ecp_irp6_postument (ini_con->ui->ECP_SECTION);
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
				// ini_con->create_edp_irp6_postument (ini_con->ui->EDP_SECTION);

				state.edp.pid = -1;
				state.edp.reader_fd = common::invalid_fd;

				state.edp.state = 0;

				for (int i = 0; i < 4; i++) {
					char tmp_string[50];
					if (i < 3) {
						sprintf(tmp_string, "preset_position_%d", i);
					} else {
						sprintf(tmp_string, "front_position", i);
					}

					if (interface.config->exists(tmp_string, state.edp.section_name)) {
						char* tmp, *tmp1;

						tmp1 = tmp
								= strdup(interface.config->value <std::string> (tmp_string, state.edp.section_name).c_str());
						char* toDel = tmp;
						for (int j = 0; j < lib::irp6p_m::NUM_OF_SERVOS; j++) {
							if (i < 3) {
								state.edp.preset_position[i][j] = strtod(tmp1, &tmp1);
							} else {
								state.edp.front_position[j] = strtod(tmp1, &tmp1);
							}
						}
						free(toDel);
					} else {
						for (int j = 0; j < lib::irp6p_m::NUM_OF_SERVOS; j++) {
							if (i < 3) {
								state.edp.preset_position[i][j] = 0.0;
							} else {
								state.edp.front_position[j] = 0.0;
								printf("nie zdefiniowano irp6p front_postion w common.ini\n");
							}
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

	} else // jesli  irp6 postument ma byc nieaktywne
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
	} // end irp6_postument

	return 1;
}

int UiRobot::manage_interface()
{

	switch (state.edp.state)
	{

		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_postument, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_postument_edp_unload, ABN_mm_irp6_postument_pre_synchro_moves, ABN_mm_irp6_postument_absolute_moves, ABN_mm_irp6_postument_relative_moves, ABN_mm_irp6_postument_tool_specification, ABN_mm_irp6_postument_preset_positions, ABN_mm_irp6_postument_kinematic, ABN_mm_irp6_postument_servo_algorithm, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_postument, ABN_mm_irp6_postument_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_postument, NULL);
			//ApModifyItemState( &all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_edp_unload, NULL);
			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_postument_pre_synchro_moves, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_postument_edp_unload, ABN_mm_irp6_postument_absolute_moves, ABN_mm_irp6_postument_relative_moves, ABN_mm_irp6_postument_tool_specification, ABN_mm_irp6_postument_preset_positions, ABN_mm_irp6_postument_kinematic, ABN_mm_irp6_postument_servo_algorithm, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_postument_edp_load, NULL);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_postument_absolute_moves, ABN_mm_irp6_postument_relative_moves, ABN_mm_irp6_postument_preset_positions, ABN_mm_irp6_postument_tool_specification, ABN_mm_irp6_postument_kinematic, ABN_mm_irp6_postument_servo_algorithm, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_postument_edp_load, ABN_mm_irp6_postument_edp_unload, NULL);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_irp6_postument_absolute_moves, ABN_mm_irp6_postument_relative_moves, ABN_mm_irp6_postument_preset_positions, ABN_mm_irp6_postument_tool_specification, ABN_mm_irp6_postument_kinematic, ABN_mm_irp6_postument_servo_algorithm, NULL);
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6_postument_edp_unload, ABN_mm_irp6_postument_pre_synchro_moves, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6_postument_edp_load, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
			}
			break;
		default:
			break;

	}

	return 1;

}

// aktualizacja ustawien przyciskow
void UiRobot::process_control_window_irp6p_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger)
{

	if (state.edp.state <= 0) {// edp wylaczone
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
		interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
	} else {
		if (state.edp.state == 1) {// edp wlaczone reader czeka na start
			wlacz_PtButton_wnd_processes_control_all_reader_start = true;
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
			interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
			interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		} else if (state.edp.state == 2) {// edp wlaczone reader czeka na stop
			wlacz_PtButton_wnd_processes_control_all_reader_stop = true;
			wlacz_PtButton_wnd_processes_control_all_reader_trigger = true;
			interface.block_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_start);
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_stop);
			interface.unblock_widget(ABW_PtButton_wnd_processes_control_irp6p_reader_trigger);
		}
	}

	state.edp.last_state = state.edp.state;
}

void UiRobot::close_all_windows()
{
	int pt_res = PtEnter(0);

	close_wnd_irp6_postument_inc(NULL, NULL, NULL);
	close_wnd_irp6_postument_int(NULL, NULL, NULL);
	close_wnd_irp6_postument_xyz_angle_axis(NULL, NULL, NULL);
	close_wnd_irp6_postument_xyz_angle_axis_ts(NULL, NULL, NULL);
	close_wnd_irp6_postument_xyz_euler_zyz(NULL, NULL, NULL);
	close_wnd_irp6_postument_xyz_euler_zyz_ts(NULL, NULL, NULL);
	close_wnd_irp6_postument_xyz_aa_relative(NULL, NULL, NULL);
	close_wnd_irp6_postument_kinematic(NULL, NULL, NULL);
	close_wnd_irp6_postument_servo_algorithm(NULL, NULL, NULL);

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
