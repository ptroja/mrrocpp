// ------------------------------------------------------------------------
//                                   hi_rydz.cc
// 
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota conveyor
// 
// Ostatnia modyfikacja: styczen 2006
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego 
// przerwania - tasmociag 
// ------------------------------------------------------------------------


#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <process.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/types.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_conveyor_effector.
#include "edp/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "edp/conveyor/hi_local.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

struct sigevent event;

extern effector* master;



volatile common::motor_data md; // Dane przesylane z/do funkcji obslugi przerwania


// ------------------------------------------------------------------------
hardware_interface::hardware_interface ( effector &_master ) : common::hardware_interface (_master), master(_master)
{
	int irq_no;     // Numer przerwania sprzetowego 
	int i;            // Zmienna pomocnicze
	WORD int_freq; // Ustawienie czestotliwosci przerwan

	// Sledzenie zera rezolwera - wylaczane 
	trace_resolver_zero = false;

	md.is_power_on = true;
	md.is_robot_blocked = false;

	// by YOYEK & 7 - nadanie odpowiednich uprawnien watkowi 
	// 	w celu umozliwienia komunikacji z magistral isa i obslugi przerwania
	ThreadCtl (_NTO_TCTL_IO, NULL);

	memset(&event, 0, sizeof(event));// by y&w
	event.sigev_notify = SIGEV_INTR;// by y&w

	if(master.test_mode) {
		irq_no = 0;    // Przerwanie od zegara o okre sie 1ms
	} else {
		irq_no = IRQ_REAL;    // Numer przerwania sprzetowego od karty ISA
	}

	// inicjacja wystawiania przerwan
	if(master.test_mode==0)
	{
		// konieczne dla skasowania przyczyny przerwania
		out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
		in16(SERVO_REPLY_STATUS_ADR); // Odczyt stanu wylacznikow
		in16(SERVO_REPLY_INT_ADR);
	}


	if ( (int_id =InterruptAttach (irq_no, int_handler, (void *) &md , sizeof(md), 0)) == -1) 
	{
		// Obsluga bledu
		perror( "Unable to attach interrupt handler: ");
	} 

	// oczekiwanie na przerwanie
	if (hi_int_wait(INT_EMPTY,0)==-1) // jesli sie nie przyjdzie na czas
	{
		// inicjacja wystawiania przerwan
		if(master.test_mode==0)
		{
			// Ustawienie czestotliwosci przerwan
			int_freq = SET_INT_FREQUENCY | INT_FREC_DIVIDER;
			out8(ADR_OF_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR);
			out16(SERVO_COMMAND_1_ADR, int_freq);
			delay(10);
			out16(SERVO_COMMAND_1_ADR, START_CLOCK_INTERRUPTS); 
		}
	}

	// robot conveyor jest domyslnie zsynchronizowany
	master.controller_state_edp_buf.is_synchronised = md.is_synchronised = true;

	// Zakaz pracy recznej we wszystkich osiach

	for ( i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++ ) 
	{
		robot_status[i].adr_offset_plus_0 = 0;
		robot_status[i].adr_offset_plus_2 = 0;
		robot_status[i].adr_offset_plus_4 = 0;
		robot_status[i].adr_offset_plus_6 = 0;
		robot_status[i].adr_offset_plus_8 = 0;
		robot_status[i].adr_offset_plus_a = 0;
		meassured_current[i] = 0;

	};

	if(master.test_mode==0) {
		/*out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR + (BYTE)i); 
		out16(SERVO_COMMAND_1_ADR,RESET_MANUAL_MODE); // Zerowanie ruchow recznych
		out16(SERVO_COMMAND_1_ADR, PROHIBIT_MANUAL_MODE); // Zabrania ruchow za pomoca przyciskow w szafie*/
		md	.card_adress=FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR;
		md	.register_adress=SERVO_COMMAND_1_ADR;
		md	.value=RESET_MANUAL_MODE;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
		md	.value=PROHIBIT_MANUAL_MODE;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
		md.value=CONVEYOR_AXE_1_MAX_CURRENT;
		hi_int_wait(INT_SINGLE_COMMAND, 2);
	}

	if(master.test_mode==0) {
		// Zerowanie licznikow polozenia wszystkich osi
		reset_counters(); 
		is_hardware_error();		
	}

	first = true; // Pierwszy krok

}; // koniec: hardware_interface::hardware_interface( ) 
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
hardware_interface::~hardware_interface ( void )    // destruktor
{
	if(master.test_mode==0)
	{
		reset_counters();
		// Zezwolenie na prace reczna 


		md	.card_adress=FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR;
		md	.register_adress=SERVO_COMMAND_1_ADR;
		md	.value=ALLOW_MANUAL_MODE;
		hi_int_wait(INT_SINGLE_COMMAND, 2);

	}
}; // end: hardware_interface::~hardware_interface() 
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
uint64_t hardware_interface::read_write_hardware ( void )
{   

	// ------------------------------------------------------------------------
	// Obsluga sprzetu: odczyt aktualnych wartosci polozenia i zapis wartosci 
	// wypelnienia PWM

	int i;

	// zapis wartosci zadanych
	for (i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++ )
	{
		md.robot_control[i].adr_offset_plus_0 = robot_control[i].adr_offset_plus_0;
	}

	// oczekiwanie na przerwanie
	hi_int_wait(INT_SERVOING, 0);

	if(master.test_mode) {
		// Tylko dla testow
		return md.hardware_error;
	}

	for (i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++ ) {

		// przepisanie wartosci pradu
		meassured_current[i] = (md.robot_status[i].adr_offset_plus_2 & 0xFF00)>>8;

		current_absolute_position[i] = md.current_absolute_position[i]; 
		current_position_inc[i] = current_absolute_position[i] - previous_absolute_position[i];
		previous_absolute_position[i] = current_absolute_position[i];
	}

	if (!trace_resolver_zero) 
		md.hardware_error &= MASK_RESOLVER_ZERO;

	return md.hardware_error;

}; // end: hardware_interface::read_write_hardware() 
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// Zerowanie licznikow polozenia wszystkich osi
void hardware_interface::reset_counters ( void ) 
{   

	for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++ )
	{

		current_absolute_position[i] =   0;
		previous_absolute_position[i] = 0;
		current_position_inc[i] = 0.0; 

		// 	in16(SERVO_REPLY_INT_ADR);

	}; // end: for   

	md	.card_adress=FIRST_SERVO_PTR + (BYTE)CONVEYOR_SERVO_NR;
	md	.register_adress=SERVO_COMMAND_1_ADR;
	md	.value=MICROCONTROLLER_MODE;
	hi_int_wait(INT_SINGLE_COMMAND, 2);
	md	.value=STOP_MOTORS;
	hi_int_wait(INT_SINGLE_COMMAND, 2);
	md	.value=RESET_MANUAL_MODE;
	hi_int_wait(INT_SINGLE_COMMAND, 2);
	md	.value=RESET_ALARM;
	hi_int_wait(INT_SINGLE_COMMAND, 2);
	md	.value=RESET_POSITION_COUNTER;
	hi_int_wait(INT_SINGLE_COMMAND, 2);


	// Dwukrotny odczyt polozenia dla wyzerowania przyrostu wynikajacego z pierwszego

	// wyzerowanie wypelnienia
	for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++ )
	{
		robot_control[i].adr_offset_plus_0 = 0x0200;
	}; // end: for  

	// wyzerowanie przyrostu pozycji
	read_write_hardware();
	read_write_hardware();
	read_write_hardware();
	// Odczyt polozenia osi slowo 32 bitowe - negacja licznikow 16-bitowych
	// out8(ADR_OF_SERVO_PTR, FIRST_SERVO_PTR);
	// out16(SERVO_COMMAND_1_ADR, RESET_POSITION_COUNTER);
	// robot_status[0].adr_offset_plus_4 = 0xFFFF ^ in16(SERVO_REPLY_POS_LOW_ADR); // Mlodsze slowo 16-bitowe
	// robot_status[0].adr_offset_plus_6 = 0xFFFF ^ in16(SERVO_REPLY_POS_HIGH_ADR);// Starsze slowo 16-bitowe 
	// printf("L=%x U=%x   \n",robot_status[0].adr_offset_plus_4, robot_status[0].adr_offset_plus_6);
}; // end: hardware_interface::reset_counters()
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
bool hardware_interface::is_hardware_error ( void) 
{ 
	bool h_error;
	WORD MASK = 0x7E00;

	h_error = false;

	// oczekiwanie na przerwanie
	hi_int_wait(INT_SINGLE_COMMAND, 0);

	for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++ ) 
	{
		if ( (md.robot_status[i].adr_offset_plus_0 ^ 0x6000) & MASK ) 
		{
			h_error = true;
			//     printf(" \n => axis= %d r210H: %x ",i,robot_status[i].adr_offset_plus_0);
		}
	} // end: for
	return h_error;
}; // end: hardware_interface::is_hardware_error ()
// ------------------------------------------------------------------------




int hardware_interface::hi_int_wait (int inter_mode, int lag)
{
	uint64_t *int_timeout;
	struct sigevent tim_event;
	int iw_ret;

	static short interrupt_error = 0;
	static short msg_send = 0;

	int_timeout=new(uint64_t);
	*int_timeout=HI_RYDZ_INTR_TIMEOUT_HIGH;
	tim_event.sigev_notify = SIGEV_UNBLOCK;

	TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_INTR ,   &tim_event, int_timeout, NULL );
	md	.interrupt_mode=inter_mode;   // przypisanie odpowiedniego trybu oprzerwania

	iw_ret=InterruptWait (0, NULL); 

	if (iw_ret==-1) { // jesli przerwanie nie przyjdzie na czas
		if (interrupt_error == 1) master.msg->message(NON_FATAL_ERROR, "Nie odebrano przerwania - sprawdz szafe");
		interrupt_error++;
		master.controller_state_edp_buf.is_wardrobe_on = false;
	} else {

		if (interrupt_error >= 1) master.msg->message("Przywrocono obsluge przerwania");
		interrupt_error = 0;
		master.controller_state_edp_buf.is_wardrobe_on = true;
		master.controller_state_edp_buf.is_power_on = md.is_power_on;
	}



	if ((interrupt_error>2) || (!master.controller_state_edp_buf.is_power_on))
	{
		if ((msg_send++) == 0) master.msg->message(NON_FATAL_ERROR, "Wylaczono moc - robot zablokowany");
		md.is_robot_blocked = true;
	}

	master.controller_state_edp_buf.is_robot_blocked = md.is_robot_blocked;

	if (lag!=0) delay(lag); // opoznienie niezbedne do przyjecia niektorych komend

	delete int_timeout;
	return iw_ret;
};

} // namespace conveyor
} // namespace edp
} // namespace mrrocpp

