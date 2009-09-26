// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota conveyor
//
// Ostatnia modyfikacja: styczen 2005
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#ifdef __QNXNTO__
#include <process.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#endif
#ifdef	linux
#include <sys/io.h>
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_conveyor_effector.
#include "edp/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "edp/conveyor/hi_conv.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {


// ------------------------------------------------------------------------
hardware_interface::hardware_interface (common::manip_and_conv_effector &_master, int _hi_irq_real,
		unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high,
		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, int* _max_current)
		: common::hardware_interface(_master, _hi_irq_real, _hi_intr_freq_divider,
		_hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset, _max_current)
{} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------



} // namespace conveyor
} // namespace edp
} // namespace mrrocpp

