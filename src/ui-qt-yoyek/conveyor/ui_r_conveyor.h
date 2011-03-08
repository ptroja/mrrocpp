// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_CONVEYOR_H
#define __UI_R_CONVEYOR_H

#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/conveyor/const_conveyor.h"

class wgt_conveyor_move;

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;

}
namespace single_motor {
class EcpRobot;
}

namespace conveyor {

//
// KLASA UiRobotConveyor
//
//


class UiRobot : public common::UiRobot
{
private:

public:

	double current_pos[lib::conveyor::NUM_OF_SERVOS];// pozycja biezaca
	double desired_pos[lib::conveyor::NUM_OF_SERVOS]; // pozycja zadana

	single_motor::EcpRobot *ui_ecp_robot;
	wgt_conveyor_move *wgt_inc;

	UiRobot(common::Interface& _interface);

	int manage_interface();
	int
			process_control_window_conveyor_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger);
	void delete_ui_ecp_robot();
	int synchronise();
	int synchronise_int();
	void edp_create();
	int edp_create_int();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

