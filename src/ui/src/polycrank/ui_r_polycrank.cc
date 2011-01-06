/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/ui_ecp_r_irp6_common.h"

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

	set_ui_state_notification(UI_N_PROCESS_CREATION);

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

					ui_ecp_robot
							= new ui::irp6::EcpRobot(*interface.config, *interface.all_ecp_msg, lib::polycrank::ROBOT_NAME);
					/*
					 ui_ecp_robot
					 = new ui::irp6::EcpRobot(*interface.config, *interface.all_ecp_msg, lib::irp6p_m::ROBOT_NAME);
					 */
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
	//Polycrank will be always in synchronise poses
	return 1;

}

/*
 int UiRobot::synchronise_int()
 {

 set_ui_state_notification(UI_N_SYNCHRONISATION);

 // wychwytania ew. bledow ECP::robot
 try {
 // dla robota irp6_on_track

 if ((state.edp.state > 0) && (state.edp.is_synchronised == false)) {
 ui_ecp_robot->ecp->synchronise();
 state.edp.is_synchronised = ui_ecp_robot->ecp->is_synchronised();
 } else {
 // 	printf("edp irp6_on_track niepowolane, synchronizacja niedozwolona\n");
 }

 } // end try
 CATCH_SECTION_UI

 // modyfikacje menu
 interface.manage_interface();

 return 1;

 }
 */

/*
 UiRobot::UiRobot(common::Interface& _interface) :
 common::UiRobot(_interface, lib::irp6::EDP_SECTION, ECP_SECTION), ui_ecp_robot(NULL),
 is_wind_polycrank_int_open(false)//, is_wind_polycrank_inc_open(false)
 {

 }*/

UiRobot::UiRobot(common::Interface& _interface) :
			common::UiRobot(_interface, lib::polycrank::EDP_SECTION, lib::polycrank::ECP_SECTION, lib::polycrank::ROBOT_NAME, lib::polycrank::NUM_OF_SERVOS, "is_polycrank_active"),
			is_wind_polycrank_int_open(false), ui_ecp_robot(NULL)
{
}

void UiRobot::close_all_windows()
{
}

/*
 UiRobot::UiRobot(common::Interface& _interface) :
 common::UiRobot(_interface, lib::irp6p_m::EDP_SECTION, lib::irp6p_m::ECP_SECTION, lib::irp6p_m::ROBOT_NAME),
 is_wind_irp6p_int_open(false), is_wind_irp6p_inc_open(false), is_wind_irp6p_xyz_euler_zyz_open(false),
 is_wind_irp6p_xyz_angle_axis_open(false), is_wind_irp6p_xyz_aa_relative_open(false),
 is_wind_irp6p_xyz_angle_axis_ts_open(false), is_wind_irp6p_xyz_euler_zyz_ts_open(false),
 is_wind_irp6p_kinematic_open(false), is_wind_irp6p_servo_algorithm_open(false), ui_ecp_robot(NULL)
 {
 }*/

int UiRobot::reload_configuration()
{
	// jesli Polycrank ma byc aktywne
	if ((state.is_active = interface.config->value <int> (activation_string)) == 1) {
		// ini_con->create_ecp_irp6_on_track (ini_con->ui->ECP_SECTION);
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
				// ini_con->create_edp_irp6_on_track (ini_con->ui->EDP_SECTION);

				state.edp.pid = -1;
				state.edp.reader_fd = lib::invalid_fd;
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
						tmp1
								= tmp
										= strdup(interface.config->value <std::string> (tmp_string, state.edp.section_name).c_str());
						char* toDel = tmp;
						for (int j = 0; j < number_of_servos; j++) {
							if (i < 3) {
								state.edp.preset_position[i][j] = strtod(tmp1, &tmp1);
							} else {
								state.edp.front_position[j] = strtod(tmp1, &tmp1);
							}
						}
						free(toDel);
					} else {
						for (int j = 0; j < number_of_servos; j++) {
							if (i < 3) {
								state.edp.preset_position[i][j] = 0.0;
							} else {
								state.edp.front_position[j] = 0.0;
								printf("nie zdefiniowano polycrank front_postion w common.ini\n");
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

	} else // jesli polycrank ma byc nieaktywne
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
	} // end polycrank

	return 1;

}

int UiRobot::manage_interface()
{
	switch (state.edp.state)
	{
		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_unload, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank, ABN_mm_polycrank_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank, NULL);

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);
				//ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);

				switch (interface.mp.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_edp_unload, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, NULL);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL,

						NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, ABN_mm_polycrank_edp_unload, NULL);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, NULL);// modyfikacja menu - ruchy reczne zakazane
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_polycrank_edp_unload, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_polycrank_edp_load, NULL);
				//ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
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
