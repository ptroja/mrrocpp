// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SMB_H
#define __UI_R_SMB_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

namespace mrrocpp {
namespace uin {
namespace common {

//
//
// KLASA UiRobotSmb
//
//


// super klasa agregujaca porozrzucane struktury

class Ui;
class ui_tfg_and_conv_robot;

class UiRobotSmb: public UiRobot {
private:

public:

	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotSmb(Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int delete_ui_ecp_robot();

};

}
} //namespace uin
} //namespace mrrocpp

#endif

