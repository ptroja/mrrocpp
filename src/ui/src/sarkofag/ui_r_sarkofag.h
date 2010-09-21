// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SARKOFAG_H
#define __UI_R_SARKOFAG_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"
#include "robot/sarkofag/const_sarkofag.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace tfg_and_conv {
class EcpRobot;
}
namespace sarkofag {

//
//
// KLASA UiRobotSarkofag
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobot : public common::UiRobot
{
private:

public:

	double sarkofag_current_pos[lib::sarkofag::NUM_OF_SERVOS];// pozycja biezaca
	double sarkofag_desired_pos[lib::sarkofag::NUM_OF_SERVOS]; // pozycja zadana


	bool is_wind_sarkofag_moves_open; // informacja czy okno ruchow
	bool is_wind_sarkofag_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte

	tfg_and_conv::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();
	int close_all_windows();
	int delete_ui_ecp_robot();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

