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
namespace uin {
namespace common {
class Ui;
}
namespace tfg_and_conv {
class EcpRobot;
}
namespace spkm {

//
//
// KLASA UiRobot
//
//


class UiRobot : public common::UiRobot
{
private:

public:

	tfg_and_conv::EcpRobot *ui_ecp_robot;

	UiRobot(common::Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int delete_ui_ecp_robot();
};

}
} //namespace uin
} //namespace mrrocpp

#endif

