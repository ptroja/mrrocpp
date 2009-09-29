// ------------------------------------------------------------------------
//							hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota polycrank
//
// ------------------------------------------------------------------------

// Klasa edp_irp6m_effector.
#include "edp/polycrank/edp_e_polycrank.h"
// Klasa hardware_interface.
#include "edp/polycrank/hi_polycrank.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

// ------------------------------------------------------------------------
hardware_interface::hardware_interface (common::manip_and_conv_effector &_master, int _hi_irq_real,
		unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high,
		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, int* _max_current)
		: common::hardware_interface(_master, _hi_irq_real, _hi_intr_freq_divider,
		_hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset, _max_current)
{} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------



} // namespace polycrank
} // namespace edp
} // namespace mrrocpp
