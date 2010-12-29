// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPKM_H
#define __UI_R_SPKM_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm {

//
//
// KLASA UiRobot
//
//

class EcpRobot;
class WndInc;

class UiRobot : public common::UiRobot
{
private:

public:

	EcpRobot *ui_ecp_robot;
	WndInc *wnd_inc;

	UiRobot(common::Interface& _interface);
	void close_all_windows();
	int reload_configuration();
	int manage_interface();
	void delete_ui_ecp_robot();
	int synchronise();
	int synchronise_int();
	int edp_create();
	int edp_create_int();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

