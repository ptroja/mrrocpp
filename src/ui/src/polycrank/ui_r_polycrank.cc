/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/polycrank/ui_r_polycrank.h"
#include "lib/robot_consts/polycrank_const.h"
#include "ui/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"


//
//
// KLASA UiRobotIrp6ot_m
//
//


UiRobotPolycrank::UiRobotPolycrank(Ui& _ui) :
	UiRobot(_ui, EDP_POLYCRANK_SECTION, ECP_POLYCRANK_SECTION), ui_ecp_robot(NULL),
			is_wind_polycrank_int_open(false),
			is_wind_polycrank_inc_open(false) {

}

int UiRobotPolycrank::reload_configuration() {
}

int UiRobotPolycrank::manage_interface() {
}

int UiRobotPolycrank::delete_ui_ecp_robot() {
	delete ui_ecp_robot;
}

