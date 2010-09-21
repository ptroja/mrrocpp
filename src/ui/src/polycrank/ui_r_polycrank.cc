/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/polycrank/ui_r_polycrank.h"
#include "robot/polycrank/const_polycrank.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace ui {
namespace polycrank {

//
//
// KLASA UiRobot
//
//


UiRobot::UiRobot(common::Interface& _interface) :
	common::UiRobot(_interface, lib::polycrank::EDP_SECTION, ECP_SECTION), ui_ecp_robot(NULL),
			is_wind_polycrank_int_open(false), is_wind_polycrank_inc_open(false)
{

}

int UiRobot::reload_configuration()
{
}

int UiRobot::manage_interface()
{
}

int UiRobot::delete_ui_ecp_robot()
{
	delete ui_ecp_robot;
}

}
} //namespace ui
} //namespace mrrocpp
