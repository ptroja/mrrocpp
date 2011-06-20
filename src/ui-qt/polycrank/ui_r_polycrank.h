// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_POLYCRANK_H
#define __UI_R_POLYCRANK_H

#include <QObject>
#include "../base/mainwindow.h"
#include "../base/interface.h"
#include "../base/ui.h"
#include "../base/ui_robot.h"
#include "robot/polycrank/const_polycrank.h"

//class wgt_spkm_inc;
class wgt_polycrank_int;

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

namespace polycrank {

//
//
// KLASA UiRobotPolycrank
//
//

class UiRobot : public common::UiRobot
{
	Q_OBJECT
private:

public:
	double current_pos[lib::polycrank::NUM_OF_SERVOS]; // pozycja biezaca
	double desired_pos[lib::polycrank::NUM_OF_SERVOS]; // pozycja zadana

	//bool is_wind_polycrank_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte

	common::EcpRobot *ui_ecp_robot;

	//wgt_spkm_inc *wgt_int;

	UiRobot(common::Interface& _interface);

	int manage_interface();
	void delete_ui_ecp_robot();
	void null_ui_ecp_robot();
	int synchronise();
	int synchronise_int();

	int create_ui_ecp_robot();

	int ui_get_edp_pid();
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);
	void make_connections();
	void setup_menubar();

private:
    QAction *actionpolycrank_Move_Joints;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

