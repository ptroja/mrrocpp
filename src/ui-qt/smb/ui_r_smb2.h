// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SMB2_H
#define __UI_R_SMB2_H

#include <QObject>
#include <QMenu>
#include "ui_r_smb.h"
#include "robot/smb/const_smb2.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}
namespace smb2 {

//
//
// KLASA UiRobot
//
//


class UiRobot : public smb::UiRobot
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

