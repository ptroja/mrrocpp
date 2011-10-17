// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6OT_M_H
#define __UI_R_IRP6OT_M_H

#include <QObject>
#include "../base/mainwindow.h"
#include "../base/interface.h"
#include "../base/ui.h"
#include "../irp6_m/ui_r_irp6_m.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

namespace Ui{
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class EcpRobot;
}
}
}

namespace mrrocpp {
namespace ui {
namespace irp6ot_m {

//
//
// KLASA UiRobotIrp6ot_m
//
//


class UiRobot : public irp6_m::UiRobot
{
	Q_OBJECT

private:

public:

	UiRobot(common::Interface& _interface);



	virtual int synchronise();

	int move_to_synchro_position();
	int move_to_front_position();
	int move_to_preset_position(int variant);

	void create_ui_ecp_robot();
	int edp_create_int_extra_operations();

	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);


	void setup_menubar();


};

}
} //namespace ui
} //namespace mrrocpp

#endif

