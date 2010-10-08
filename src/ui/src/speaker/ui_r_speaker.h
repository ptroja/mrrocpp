// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPEAKER_H
#define __UI_R_SPEAKER_H

#include "ui/src/ui.h"
#include "ui/src/ui_robot.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace speaker {

//
//
// KLASA UiRobot
//
//


class EcpRobot;

class UiRobot : public common::UiRobot
{
private:

public:

	bool is_wind_speaker_play_open; // informacja czy okno odtwarzania dzwiekow jest otwarte
	EcpRobot *ui_ecp_robot;

	UiRobot(common::Interface& _interface);
	int reload_configuration();
	int manage_interface();

	void close_all_windows();
	void delete_ui_ecp_robot();
};

}
} //namespace ui
} //namespace mrrocpp

#endif

