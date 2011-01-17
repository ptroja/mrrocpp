// ------------------------------------------------------------------------
//                            hi_polycrank.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota polycrank
//
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

// Klasa edp_polycrank_effector.
#include "robot/polycrank/edp_e_polycrank.h"
// Klasa hardware_interface.
#include "robot/polycrank/hi_polycrank.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

// ------------------------------------------------------------------------
//hardware_interface::hardware_interface(common::motor_driven_effector &_master) :
//	hi_moxa::HI_moxa(_master, mrrocpp::lib::polycrank::FIRST_MOXA_PORT_NUM, mrrocpp::lib::polycrank::LAST_MOXA_PORT_NUM))
//{
//} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------


} // namespace polycrank
} // namespace edp
} // namespace mrrocpp

