// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SHEAD_H
#define __UI_R_SHEAD_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotShead
//
//


// super klasa agregujaca porozrzucane struktury

class Ui;
class ui_tfg_and_conv_robot;

class UiRobotShead: public UiRobot {
private:

public:

	ui_tfg_and_conv_robot *ui_ecp_robot;

	UiRobotShead(Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int delete_ui_ecp_robot();
};

#endif

