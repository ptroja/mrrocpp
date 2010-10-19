// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 mechatronika
//
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

// Klasa edp_irp6m_effector.
#include "robot/sarkofag/edp_e_sarkofag.h"
// Klasa hardware_interface.
#include "robot/sarkofag/hi_sarkofag.h"

namespace mrrocpp {
namespace edp {
namespace sarkofag {

// ------------------------------------------------------------------------
hardware_interface::hardware_interface(common::motor_driven_effector &_master) :
	hi_moxa::HI_moxa(_master, mrrocpp::lib::sarkofag::FIRST_MOXA_PORT_NUM, mrrocpp::lib::sarkofag::LAST_MOXA_PORT_NUM)
{
} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------


} // namespace sarkofag
} // namespace edp
} // namespace mrrocpp
