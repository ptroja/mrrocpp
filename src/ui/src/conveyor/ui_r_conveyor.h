// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_CONVEYOR_H
#define __UI_R_CONVEYOR_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "robot/conveyor/const_conveyor.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;

}
namespace tfg_and_conv {
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

	double conveyor_current_pos[lib::conveyor::NUM_OF_SERVOS];// pozycja biezaca
	double conveyor_desired_pos[lib::conveyor::NUM_OF_SERVOS]; // pozycja zadana

	bool is_wind_conv_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_conveyor_moves_open; // informacja czy okno ruchow dla robota conveyor

	tfg_and_conv::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();
	int
			process_control_window_conveyor_section_init(bool &wlacz_PtButton_wnd_processes_control_all_reader_start, bool &wlacz_PtButton_wnd_processes_control_all_reader_stop, bool &wlacz_PtButton_wnd_processes_control_all_reader_trigger);
	void close_all_windows();
	void delete_ui_ecp_robot();

};

}
} //namespace ui
} //namespace mrrocpp

#endif

