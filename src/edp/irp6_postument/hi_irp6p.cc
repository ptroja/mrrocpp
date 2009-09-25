// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 postument
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

// Klasa edp_irp6p_effector.
#include "edp/irp6_postument/edp_irp6p_effector.h"
// Klasa hardware_interface.
#include "edp/irp6_postument/hi_irp6p.h"

namespace mrrocpp {
namespace edp {
namespace irp6p {

// ------------------------------------------------------------------------
hardware_interface::hardware_interface (common::manip_and_conv_effector &_master, int _hi_irq_real,
		unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high,
		unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset)
		: common::hardware_interface(_master, _hi_irq_real, _hi_intr_freq_divider,
		_hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset)
{
	// tablica pradow maksymalnych dla poszczegolnych osi
	int max_current [IRP6_POSTUMENT_NUM_OF_SERVOS] = {
			IRP6_POSTUMENT_AXIS_1_MAX_CURRENT, IRP6_POSTUMENT_AXIS_2_MAX_CURRENT,
			IRP6_POSTUMENT_AXIS_3_MAX_CURRENT, IRP6_POSTUMENT_AXIS_4_MAX_CURRENT,
			IRP6_POSTUMENT_AXIS_5_MAX_CURRENT, IRP6_POSTUMENT_AXIS_6_MAX_CURRENT,
			IRP6_POSTUMENT_AXIS_7_MAX_CURRENT
	};

	// Sledzenie zera rezolwera - wylaczane
	trace_resolver_zero = false;

	irq_data.md.is_power_on = true;
	irq_data.md.is_robot_blocked = false;

	if(master.test_mode) {
		// domyslnie robot jest zsynchronizowany
		irq_data.md.is_synchronised = true;

	    fprintf(stderr, "Blocking signal %d\n", SIGRTMIN);
	    if (sigemptyset (&mask) == -1) {
	    	perror("sigemptyset()");
	    }
	    if (sigaddset (&mask, SIGRTMIN) == -1) {
	    	perror("sigaddset()");
	    }

	    /* Create the timer */
	    struct sigevent sev;
	    sev.sigev_notify = SIGEV_SIGNAL;
	    sev.sigev_signo = SIGRTMIN;
	    sev.sigev_value.sival_ptr = &timerid;
	    if (timer_create (CLOCK_REALTIME, &sev, &timerid) == -1) {
	    	perror("timer_create()");
	    }

	    /* Start the timer */
	    struct itimerspec its;
	    its.it_value.tv_sec = 0;
	    its.it_value.tv_nsec = 1000000; // 1kHz
	    its.it_interval.tv_sec = its.it_value.tv_sec;
	    its.it_interval.tv_nsec = its.it_value.tv_nsec;

	    if (timer_settime (timerid, 0, &its, NULL) == -1) {
	    	perror("timer_settime()");
	    }
	} else {
		// domyslnie robot nie jest zsynchronizowany
		irq_data.md.is_synchronised = false;

#ifdef __QNXNTO__
		// by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi
		// 	w celu umozliwienia komunikacji z magistral isa i obslugi przerwania
		ThreadCtl (_NTO_TCTL_IO, NULL);

		if (mmap_device_io(0xC, (SERVO_COMMAND1_ADR + ISA_CARD_OFFSET)) == MAP_DEVICE_FAILED) {
			perror("mmap_device_io");
		}

		if (mmap_device_io(1, (ADR_OF_SERVO_PTR + ISA_CARD_OFFSET)) == MAP_DEVICE_FAILED) {
			perror("mmap_device_io");
		}

		memset(&irq_data.event, 0, sizeof(irq_data.event));// by y&w
		irq_data.event.sigev_notify = SIGEV_INTR;// by y&w
#endif

		irq_data.md.interrupt_mode=INT_EMPTY;

		// konieczne dla skasowania przyczyny przerwania
		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), INTERRUPT_GENERATOR_SERVO_PTR);
		in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

#ifdef __QNXNTO__
		int irq_no = IRQ_REAL;   // Numer przerwania sprzetowego od karty ISA
		if ((int_id = InterruptAttach (irq_no, int_handler, (void *) &irq_data, sizeof(irq_data), 0)) == -1)
		{
			perror("Unable to attach interrupt handler");
		}
#endif
	}

	// oczekiwanie na przerwanie
	if (hi_int_wait(INT_EMPTY, 0)==-1) // jesli nie przyjdzie na czas
	{
		// inicjacja wystawiania przerwan
		if(master.test_mode==0)
		{
			// Ustawienie czestotliwosci przerwan
			uint16_t int_freq = SET_INT_FREQUENCY | INT_FREC_DIVIDER;
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), INTERRUPT_GENERATOR_SERVO_PTR);
			out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), int_freq);
			delay(10);
			out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), START_CLOCK_INTERRUPTS);
		}
	}

	master.controller_state_edp_buf.is_synchronised = irq_data.md.is_synchronised;

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

		if(master.test_mode==0) {
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
	}

	if(master.test_mode==0) {
		// Zerowanie licznikow polozenia wszystkich osi
		reset_counters();
		is_hardware_error();
	}

	first = true; // Pierwszy krok
} // koniec: hardware_interface::hardware_interface( )
// ------------------------------------------------------------------------



} // namespace irp6p
} // namespace edp
} // namespace mrrocpp
