/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui/src/ui_robot.h"
#include "ui/src/ui_class.h"

namespace mrrocpp {
namespace ui {
namespace common {

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(Interface& _interface, const std::string edp_section_name, const std::string ecp_section_name) :
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
	tid = new feb_thread(eb);
}

void UiRobot::abort_thread()
{
	delete tid;
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

int UiRobot::close_all_windows()
{

	return 1;

}

int UiRobot::EDP_slay_int()

{

	// dla robota bird_hand
	if (state.edp.state > 0) { // jesli istnieje EDP
		if (state.edp.reader_fd >= 0) {
			if (name_close(state.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n", __FILE__, __LINE__, strerror(errno));
			}
		}
		close_all_windows();

		delete_ui_ecp_robot();
		state.edp.state = 0; // edp wylaczone
		state.edp.is_synchronised = false;

		state.edp.pid = -1;
		state.edp.reader_fd = -1;

		abort_thread();

	}

	// modyfikacja menu

	interface.manage_interface();

	return 1;

}

}
} //namespace ui
} //namespace mrrocpp
