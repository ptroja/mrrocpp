// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 on_track
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

// Klasa edp_irp6ot_effector.
#include "edp/irp6_on_track/edp_irp6ot_effector.h"

// Klasa hardware_interface.
#include "edp/irp6_on_track/hi_irp6ot.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot {

// ------------------------------------------------------------------------
hardware_interface::hardware_interface (common::manip_and_conv_effector &_master, int _hi_irq_real,
		unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high,
		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset)
		: common::hardware_interface(_master, _hi_irq_real, _hi_intr_freq_divider,
		_hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset)
{
	// tablica pradow maksymalnych d;a poszczegolnych osi
	int max_current [IRP6_ON_TRACK_NUM_OF_SERVOS] = {
			IRP6_ON_TRACK_AXIS_1_MAX_CURRENT, IRP6_ON_TRACK_AXIS_2_MAX_CURRENT,
			IRP6_ON_TRACK_AXIS_3_MAX_CURRENT, IRP6_ON_TRACK_AXIS_4_MAX_CURRENT,
			IRP6_ON_TRACK_AXIS_5_MAX_CURRENT, IRP6_ON_TRACK_AXIS_6_MAX_CURRENT,
			IRP6_ON_TRACK_AXIS_7_MAX_CURRENT, IRP6_ON_TRACK_AXIS_8_MAX_CURRENT
	};



	master.controller_state_edp_buf.is_synchronised = irq_data.md.is_synchronised;

	// Zakaz pracy recznej we wszystkich osiach

	if(master.test_mode==0) {
		for (int i = 0; i < master.number_of_servos; i++ )
		{
			/*
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET),RESET_MANUAL_MODE); // Zerowanie ruchow recznych
			out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), PROHIBIT_MANUAL_MODE); // Zabrania ruchow za pomoca przyciskow w szafie
			*/
			irq_data.md.card_adress=FIRST_SERVO_PTR + (uint8_t)i;
			irq_data.md.register_adress=(SERVO_COMMAND1_ADR + ISA_CARD_OFFSET);
			irq_data.md.value=RESET_MANUAL_MODE;
			hi_int_wait(INT_SINGLE_COMMAND, 2);
			irq_data.md.value=PROHIBIT_MANUAL_MODE;
			hi_int_wait(INT_SINGLE_COMMAND, 2);
			irq_data.md.value=max_current[i];
			hi_int_wait(INT_SINGLE_COMMAND, 2);

		}
		// Zerowanie licznikow polozenia wszystkich osi
		reset_counters();
		is_hardware_error();
	}

	first = true; // Pierwszy krok
} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
uint64_t hardware_interface::read_write_hardware ( void )
{

	// ------------------------------------------------------------------------
	// Obsluga sprzetu: odczyt aktualnych wartosci polozenia i zapis wartosci
	// wypelnienia PWM

	// zapis wartosci zadanych
	for (int i = 0; i < master.number_of_servos; i++ )
	{
		irq_data.md.robot_control[i].adr_offset_plus_0 = robot_control[i].adr_offset_plus_0;
	}

	// oczekiwanie na przerwanie
	hi_int_wait(INT_SERVOING, 0);

	if(master.test_mode) {
		// Tylko dla testow
		return irq_data.md.hardware_error;
	}

	//	 printf("hi rydz 1 current_absolute_position: %d, hex: %x\n", irq_data.md.current_absolute_position[5], irq_data.md.current_absolute_position[5] ); // debug

	for (int i = 0; i < master.number_of_servos; i++ ) {

		// przepisanie wartosci pradu
		if (i<6) // osie rezolwerowe
		{
			meassured_current[i] = irq_data.md.robot_status[i].adr_offset_plus_0 & 0x00FF;
		} else {
			meassured_current[i] = (irq_data.md.robot_status[i].adr_offset_plus_2 & 0xFF00)>>8;
		}

		current_absolute_position[i] = irq_data.md.current_absolute_position[i];
		current_position_inc[i] = current_absolute_position[i] - previous_absolute_position[i];
		previous_absolute_position[i] = current_absolute_position[i];
	}

	if (!trace_resolver_zero)
	{
		//	printf("read_write_hardware: w mask resolver_zero\n");
		irq_data.md.hardware_error &= lib::MASK_RESOLVER_ZERO;
	}

	return irq_data.md.hardware_error;
} // end: hardware_interface::read_write_hardware()
// ------------------------------------------------------------------------



void hardware_interface::finish_synchro (int drive_number)
{
	trace_resolver_zero = false;

	// Zakonczyc sledzenie zera rezolwera i przejdz do trybu normalnej pracy
	irq_data.md.card_adress = FIRST_SERVO_PTR + (uint8_t)drive_number;
	irq_data.md.register_adress = (SERVO_COMMAND1_ADR + ISA_CARD_OFFSET);
	irq_data.md.value = FINISH_SYNCHRO;
	hi_int_wait(INT_SINGLE_COMMAND, 2);

	// by Y - UWAGA NIE WIEDZIEC CZEMU BEZ TEGO NIE ZAWSZE DZIALAJA RUCHY NA OSI PO SYNCHRONIZACJi
	if (drive_number>5)
	{
		irq_data.md.value = MICROCONTROLLER_MODE;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
	}

}  // end: finish_synchro()

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp

