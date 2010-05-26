/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */
#include "ui/ui_robot.h"
#include "ui/ui_ecp.h"
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
