// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 postument
//
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

// Klasa edp_irp6p_effector.
#include "robot/irp6p_m/edp_irp6p_m_effector.h"
// Klasa hardware_interface.
#include "robot/irp6p_m/hi_irp6p_m.h"

namespace mrrocpp {
namespace edp {
namespace irp6p_m {

// ------------------------------------------------------------------------
//hardware_interface::hardware_interface(common::motor_driven_effector &_master) :
//	hi_moxa::HI_moxa(_master, mrrocpp::lib::irp6p_m::FIRST_MOXA_PORT_NUM, mrrocpp::lib::irp6p_m::LAST_MOXA_PORT_NUM)
//{
//}
// ------------------------------------------------------------------------



} // namespace irp6p
} // namespace edp
} // namespace mrrocpp
