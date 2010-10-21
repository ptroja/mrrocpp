// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_POLYCRANK_H
#define __UI_R_POLYCRANK_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace irp6 {
class EcpRobot;
}
namespace polycrank {

//
//
// KLASA UiRobotIrp6ot_m
//
//


class UiRobot : public common::UiRobot
{
private:

public:

	bool is_wind_polycrank_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_polycrank_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte


	irp6::EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();
	void delete_ui_ecp_robot();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

