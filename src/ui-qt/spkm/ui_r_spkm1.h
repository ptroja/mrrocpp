// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPKM1_H
#define __UI_R_SPKM1_H

#include <QObject>
#include <QMenu>
#include "ui_r_spkm.h"
#include "robot/spkm/const_spkm1.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace spkm1 {

//
//
// KLASA UiRobot
//
//


class UiRobot : public spkm::UiRobot
{
Q_OBJECT

public:

	UiRobot(common::Interface& _interface);

	int create_ui_ecp_robot();
	void setup_menubar();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

