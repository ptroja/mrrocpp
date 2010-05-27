/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/polycrank/ui_r_polycrank.h"
#include "lib/robot_consts/polycrank_const.h"
#include "ui/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

extern Ui ui;

// extern ui_state_def ui_state;

//
//
// KLASA UiRobotIrp6ot_m
//
//


UiRobotPolycrank::UiRobotPolycrank() :
	UiRobot(EDP_POLYCRANK_SECTION, ECP_POLYCRANK_SECTION), ui_ecp_robot(NULL),
			is_wind_polycrank_int_open(false),
			is_wind_polycrank_inc_open(false) {

}

int UiRobotPolycrank::reload_configuration() {
}

int UiRobotPolycrank::manage_interface() {
}

bool UiRobotPolycrank::pulse_reader_polycrank_start_exec_pulse() {

	if (ui.irp6m_m.state.edp.state == 1) {
		pulse_reader_execute(ui.irp6m_m.state.edp.reader_fd, READER_START, 0);
		ui.irp6m_m.state.edp.state = 2;
		return true;
	}

	return false;
}

bool UiRobotPolycrank::pulse_reader_polycrank_stop_exec_pulse() {

	if (ui.irp6m_m.state.edp.state == 2) {
		pulse_reader_execute(ui.irp6m_m.state.edp.reader_fd, READER_STOP, 0);
		ui.irp6m_m.state.edp.state = 1;
		return true;
	}

	return false;
}

bool UiRobotPolycrank::pulse_reader_polycrank_trigger_exec_pulse() {

	if (ui.irp6m_m.state.edp.state == 2) {
		pulse_reader_execute(ui.irp6m_m.state.edp.reader_fd, READER_TRIGGER, 0);

		return true;
	}

	return false;
}

