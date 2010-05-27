/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui/ui_robot.h"
#include "ui/ui_class.h"

extern Ui ui;

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(const std::string edp_section_name,
		const std::string ecp_section_name) :
	tid(NULL) {
	state.edp.section_name = edp_section_name;
	state.ecp.section_name = ecp_section_name;
	state.edp.state = -1; // edp nieaktywne
	state.edp.last_state = -1; // edp nieaktywne
	state.ecp.trigger_fd = -1;
	state.edp.is_synchronised = false; // edp nieaktywne

}

void UiRobot::create_thread() {
	tid = new feb_thread(eb);
}

void UiRobot::abort_thread() {
	delete tid;
}

bool UiRobot::pulse_reader_start_exec_pulse() {

	if (state.edp.state == 1) {
		ui.pulse_reader_execute(state.edp.reader_fd, READER_START, 0);
		state.edp.state = 2;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_stop_exec_pulse() {

	if (state.edp.state == 2) {
		ui.pulse_reader_execute(state.edp.reader_fd, READER_STOP, 0);
		state.edp.state = 1;
		return true;
	}

	return false;
}

bool UiRobot::pulse_reader_trigger_exec_pulse() {

	if (state.edp.state == 2) {
		ui.pulse_reader_execute(state.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}

	return false;
}
