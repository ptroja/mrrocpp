// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_POLYCRANK_H
#define __UI_R_POLYCRANK_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotIrp6ot_m
//
//


// super klasa agregujaca porozrzucane struktury

class Ui;
class ui_irp6_common_robot;

class UiRobotPolycrank: public UiRobot {
private:

public:

	bool is_wind_polycrank_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_polycrank_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte


	ui_irp6_common_robot *ui_ecp_robot;

	UiRobotPolycrank(Ui& _ui);
	int reload_configuration();
	int manage_interface();
	int delete_ui_ecp_robot();

};

#endif

