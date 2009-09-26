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
		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset)
		: common::hardware_interface(_master, _hi_irq_real, _hi_intr_freq_divider,
		_hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset)
{


	// robot conveyor jest zawsze zsynchronizowany
	master.controller_state_edp_buf.is_synchronised = irq_data.md.is_synchronised = true;

	// Zakaz pracy recznej we wszystkich osiach

	for (int i = 0; i < master.number_of_servos; i++ )
	{
		robot_status[i].adr_offset_plus_0 = 0;
		robot_status[i].adr_offset_plus_2 = 0;
		robot_status[i].adr_offset_plus_4 = 0;
		robot_status[i].adr_offset_plus_6 = 0;
		robot_status[i].adr_offset_plus_8 = 0;
		robot_status[i].adr_offset_plus_a = 0;
		meassured_current[i] = 0;

	}

	if(master.test_mode==0) {
		/*out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
		out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET),RESET_MANUAL_MODE); // Zerowanie ruchow recznych
		out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), PROHIBIT_MANUAL_MODE); // Zabrania ruchow za pomoca przyciskow w szafie*/
		irq_data.md.card_adress=FIRST_SERVO_PTR;
		irq_data.md.register_adress=(SERVO_COMMAND1_ADR + ISA_CARD_OFFSET);
		irq_data.md.value=RESET_MANUAL_MODE;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
		irq_data.md.value=PROHIBIT_MANUAL_MODE;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
		irq_data.md.value=CONVEYOR_AXIS_1_MAX_CURRENT;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
	}

	if(master.test_mode==0) {
		// Zerowanie licznikow polozenia wszystkich osi
		reset_counters();
		is_hardware_error();
	}

	first = true; // Pierwszy krok
} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------



} // namespace conveyor
} // namespace edp
} // namespace mrrocpp

