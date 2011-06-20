/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_shead1.h"
#include "ui_ecp_r_shead1.h"

namespace mrrocpp {
namespace ui {
namespace shead1 {

UiRobot::UiRobot(common::Interface& _interface) :
	shead::UiRobot(_interface, lib::shead1::ROBOT_NAME)
{

}

int UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new EcpRobot(*this);
	return 1;
}

}
} //namespace ui
} //namespace mrrocpp

