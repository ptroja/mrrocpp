/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui_robot.h"
#include "interface.h"
/* TR
 #include "ui/src/wnd_base.h"
 */
#include "base/lib/messip/messip_dataport.h"

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

namespace mrrocpp {
namespace ui {
namespace common {

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(Interface& _interface, const std::string & edp_section_name, const std::string & ecp_section_name, lib::robot_name_t _robot_name, int _number_of_servos, const std::string & _activation_string) :
	interface(_interface), tid(NULL), eb(_interface), robot_name(_robot_name), number_of_servos(_number_of_servos)
{
	activation_string = _activation_string;

	state.edp.section_name = edp_section_name;
	state.ecp.section_name = ecp_section_name;
	state.edp.state = -1; // edp nieaktywne
	state.edp.last_state = -2; // edp nieokreslone
	state.ecp.trigger_fd = lib::invalid_fd;
	state.edp.is_synchronised = false; // edp nieaktywne
}

void UiRobot::create_thread()
{
	assert(tid == NULL);
	tid = new feb_thread(eb);
}

void UiRobot::close_all_windows()
{

	BOOST_FOREACH(const common::WndBase_pair_t & window_node, wndbase_m)
				{
					window_node.second->close();
				}

}

void UiRobot::abort_thread()
{
	assert(tid);
	delete tid;
	tid = NULL;
}

void UiRobot::connect_to_reader()
{
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia

	while ((state.edp.reader_fd = messip::port_connect(state.edp.network_reader_attach_point)) == lib::invalid_fd) {
		if ((tmp++) < lib::CONNECT_RETRY) {
			usleep(lib::CONNECT_DELAY);
		} else {
			perror("blad odwolania do READER");
			break;
		}
	}
}

bool UiRobot::pulse_reader_start_exec_pulse()
{
	if (state.edp.state == 1) {
		pulse_reader_execute(READER_START, 0);
		state.edp.state = 2;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_stop_exec_pulse()
{
	if (state.edp.state == 2) {
		pulse_reader_execute(READER_STOP, 0);
		state.edp.state = 1;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_trigger_exec_pulse()
{
	if (state.edp.state == 2) {
		pulse_reader_execute(READER_TRIGGER, 0);

		return true;
	}

	return false;
}

void UiRobot::pulse_reader_execute(int code, int value)
{

	if (messip::port_send_pulse(state.edp.reader_fd, code, value)) {
		perror("Blad w wysylaniu pulsu do redera");
	}
}

void UiRobot::connect_to_ecp_pulse_chanell()
{
	short tmp = 0;
	// kilka sekund  (~1) na otworzenie urzadzenia
	// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem

	/*
	 ualarm(ui::common::SIGALRM_TIMEOUT, 0);
	 */

	while ((state.ecp.trigger_fd = messip::port_connect(state.ecp.network_trigger_attach_point)) == NULL

	) {
		if (errno == EINTR)
			break;
		if ((tmp++) < lib::CONNECT_RETRY) {
			usleep(lib::CONNECT_DELAY);
		} else {
			perror("blad odwolania do ECP_TRIGGER");
		}
	}

	/*
	 // odwolanie alarmu
	 ualarm((useconds_t)(0), 0);
	 */
}

void UiRobot::pulse_ecp_execute(int code, int value)
{

	if (messip::port_send_pulse(state.ecp.trigger_fd, code, value))

	{
		fprintf(stderr, "Blad w wysylaniu pulsu do ecp error: %s \n", strerror(errno));
		delay(1000);
	}
}

void UiRobot::pulse_ecp()
{
	if (state.edp.is_synchronised) { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (state.ecp.trigger_fd == lib::invalid_fd) {
			connect_to_ecp_pulse_chanell();
		}

		if (state.ecp.trigger_fd != lib::invalid_fd) {
			pulse_ecp_execute(ECP_TRIGGER, 1);
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}
}

bool UiRobot::deactivate_ecp_trigger()
{

	if (state.is_active) {
		if (state.ecp.trigger_fd != lib::invalid_fd) {

			if (messip::port_disconnect(state.ecp.trigger_fd) != 0) {
				fprintf(stderr, "UIRobot::ECP trigger @%s:%d: messip::port_disconnect(): %s\n", __FILE__, __LINE__, strerror(errno));
			}

		}
		state.ecp.trigger_fd = lib::invalid_fd;
		state.ecp.pid = -1;
		return true;
	}

	return false;
}

void UiRobot::EDP_slay_int()
{
	// dla robota bird_hand
	if (state.edp.state > 0) { // jesli istnieje EDP
		if (state.edp.reader_fd != lib::invalid_fd) {

			if (messip::port_disconnect(state.edp.reader_fd) != 0) {
				fprintf(stderr, "UIRobot::EDP_slay_int@%s:%d: messip::port_disconnect(): %s\n", __FILE__, __LINE__, strerror(errno));
			}

		}
		state.edp.reader_fd = lib::invalid_fd;

		close_all_windows();

		delete_ui_ecp_robot();
		state.edp.state = 0; // edp wylaczone
		state.edp.is_synchronised = false;

		state.edp.pid = -1;

		abort_thread();
	}

	// modyfikacja menu

	interface.manage_interface();
}

// ustala stan wszytkich EDP

bool UiRobot::check_synchronised_and_loaded()
{
	return (((state.edp.state > 0) && (state.edp.is_synchronised)));

}

int UiRobot::move_to_synchro_position()
{
	return 1;
}

int UiRobot::move_to_front_position()
{
	return 1;
}

int UiRobot::move_to_preset_position(int variant)
{
	return 1;
}

int UiRobot::reload_configuration()
{

	//	printf("final_position: %lf, %lf, %lf, %lf, %lf, %lf\n ", final_position[0], final_position[1], final_position[2], final_position[3], final_position[4], final_position[5]);

	// jesli IRP6 on_track ma byc aktywne
	if ((state.is_active = interface.config->value <int> ("is_active", state.edp.section_name)) == 1) {
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
						sprintf(tmp_string, "front_position");
					}

					if (interface.config->exists(tmp_string, state.edp.section_name)) {

						std::string text(interface.config->value <std::string> (tmp_string, state.edp.section_name));

						boost::char_separator <char> sep(" ");
						boost::tokenizer <boost::char_separator <char> > tokens(text, sep);

						int j = 0;
						BOOST_FOREACH(std::string t, tokens)
									{

										if (i < 3) {
											//value = boost::lexical_cast<double>(my_string);

											state.edp.preset_position[i][j] = boost::lexical_cast <double>(t);
										} else {
											state.edp.front_position[j] = boost::lexical_cast <double>(t);
										}

										if (j == number_of_servos) {
											break;
										}
										j++;
									}

					} else {
						for (int j = 0; j < number_of_servos; j++) {
							if (i < 3) {
								state.edp.preset_position[i][j] = 0.0;
							} else {
								state.edp.front_position[j] = 0.0;
								printf("nie zdefiniowano front_position w common.ini\n");
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

				if (!interface.config->exists("node_name", state.edp.section_name)) {
					state.edp.node_name = "localhost";
				} else {
					state.edp.node_name = interface.config->value <std::string> ("node_name", state.edp.section_name);
				}
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
	} // end irp6_on_track

	return 1;
}

}
} //namespace ui
} //namespace mrrocpp
