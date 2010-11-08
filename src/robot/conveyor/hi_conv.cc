// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota conveyor
//
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

// Klasa edp_conveyor_effector.
#include "robot/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "robot/conveyor/hi_conv.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

// ------------------------------------------------------------------------
//hardware_interface::hardware_interface(common::motor_driven_effector &_master) :
//	hi_moxa::HI_moxa(_master, mrrocpp::lib::conveyor::FIRST_MOXA_PORT_NUM, mrrocpp::lib::conveyor::LAST_MOXA_PORT_NUM))
//{
//} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------


} // namespace conveyor
} // namespace edp
} // namespace mrrocpp

