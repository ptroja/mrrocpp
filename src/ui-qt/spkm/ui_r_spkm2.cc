/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_spkm2.h"
#include "ui_ecp_r_spkm2.h"

namespace mrrocpp {
namespace ui {
namespace spkm2 {

UiRobot::UiRobot(common::Interface& _interface) :
		spkm::UiRobot(_interface, lib::spkm2::ROBOT_NAME)
{
	add_wgt <wgt_spkm_inc>(spkm::WGT_SPKM_INC, "Spkm2 inc");
	add_wgt <wgt_spkm_int>(spkm::WGT_SPKM_INT, "Spkm2 int");
	add_wgt <wgt_spkm_ext>(spkm::WGT_SPKM_EXT, "Spkm2 ext");
}

void UiRobot::create_ui_ecp_robot()
{
	ui_ecp_robot = new EcpRobot(*this);
//	return 1;
}

void UiRobot::setup_menubar()
{
	spkm::UiRobot::setup_menubar();
	robot_menu->setTitle(QApplication::translate("MainWindow", "Sp&km2", 0, QApplication::UnicodeUTF8));
}

}
} //namespace ui
} //namespace mrrocpp

