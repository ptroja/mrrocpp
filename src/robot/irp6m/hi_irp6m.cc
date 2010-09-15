// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 mechatronika
//
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

// Klasa edp_irp6m_effector.
#include "robot/irp6m/edp_irp6m_effector.h"
// Klasa hardware_interface.
#include "robot/irp6m/hi_irp6m.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

// ------------------------------------------------------------------------
hardware_interface::hardware_interface (common::motor_driven_effector &_master, int _hi_irq_real,
		unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high,
		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, const int _max_current[])
		: hi_rydz::HI_rydz(_master, _hi_irq_real, _hi_intr_freq_divider,
		_hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset, _max_current)
{} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------



} // namespace irp6m
} // namespace edp
} // namespace mrrocpp
