/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui/src/ui_robot.h"
#include "ui/src/ui_class.h"

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

namespace mrrocpp {
namespace ui {
namespace common {

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(Interface& _interface, const std::string & edp_section_name, const std::string & ecp_section_name) :
	interface(_interface), tid(NULL)
{
	state.edp.section_name = edp_section_name;
	state.ecp.section_name = ecp_section_name;
	state.edp.state = -1; // edp nieaktywne
	state.edp.last_state = -1; // edp nieaktywne
	state.ecp.trigger_fd = -1;
	state.edp.is_synchronised = false; // edp nieaktywne
}

void UiRobot::create_thread()
{
	assert(tid == NULL);
	tid = new feb_thread(eb);
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

	while ((state.edp.reader_fd =
#if !defined(USE_MESSIP_SRR)
			name_open(state.edp.network_reader_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0
#else
			messip::port_connect(state.edp.network_reader_attach_point)) == NULL
#endif
			) {
		if ((tmp++) < lib::CONNECT_RETRY) {
			delay(lib::CONNECT_DELAY);
		} else {
			perror("blad odwolania do READER");
			break;
		}
	}
}

bool UiRobot::pulse_reader_start_exec_pulse()
{
	if (state.edp.state == 1) {
		interface.pulse_reader_execute(state.edp.reader_fd, READER_START, 0);
		state.edp.state = 2;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_stop_exec_pulse()
{
	if (state.edp.state == 2) {
		interface.pulse_reader_execute(state.edp.reader_fd, READER_STOP, 0);
		state.edp.state = 1;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_trigger_exec_pulse()
{
	if (state.edp.state == 2) {
		interface.pulse_reader_execute(state.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}

	return false;
}

void UiRobot::pulse_ecp()
{
	char pulse_code = ECP_TRIGGER;
	long pulse_value = 1;

	if (state.edp.is_synchronised) { // o ile ECP dziala (sprawdzanie poprzez dzialanie odpowiedniego EDP)
		if (state.ecp.trigger_fd < 0) {

			short tmp = 0;
			// kilka sekund  (~1) na otworzenie urzadzenia
			// zabezpieczenie przed zawieszeniem poprzez wyslanie sygnalu z opoznieniem

			ualarm(ui::common::SIGALRM_TIMEOUT, 0);
			while ((state.ecp.trigger_fd
					= name_open(state.ecp.network_trigger_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0) {
				if (errno == EINTR)
					break;
				if ((tmp++) < lib::CONNECT_RETRY) {
					delay(lib::CONNECT_DELAY);
				} else {
					perror("blad odwolania do ECP_TRIGGER");
				}
			}
			// odwolanie alarmu
			ualarm((useconds_t) (0), 0);
		}

		if (state.ecp.trigger_fd >= 0) {
			if (MsgSendPulse(state.ecp.trigger_fd, sched_get_priority_min(SCHED_FIFO), pulse_code, pulse_value) == -1) {

				fprintf(stderr, "Blad w wysylaniu pulsu do ecp error: %s \n", strerror(errno));
				delay(1000);
			}
		} else {
			printf("W PULS ECP:  BLAD name_open \n");
		}
	}
}

void UiRobot::close_all_windows()
{
}

void UiRobot::EDP_slay_int()
{
	// dla robota bird_hand
	if (state.edp.state > 0) { // jesli istnieje EDP
#if !defined(USE_MESSIP_SRR)
		if (state.edp.reader_fd >= 0) {
			if (name_close(state.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n", __FILE__, __LINE__, strerror(errno));
			}
		}
		state.edp.reader_fd = -1;
#else
		if (state.edp.reader_fd ) {
			if (messip::port_disconnect(state.edp.reader_fd) != 0) {
				fprintf(stderr, "UIRobot::EDP_slay_int@%s:%d: messip::port_disconnect(): %s\n", __FILE__, __LINE__, strerror(errno));
			}
		}
		state.edp.reader_fd = NULL;
#endif
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

}
} //namespace ui
} //namespace mrrocpp
