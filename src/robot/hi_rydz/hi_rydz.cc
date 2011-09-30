// ------------------------------------------------------------------------
//                                   hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota conveyor
//
// Ostatnia modyfikacja: styczen 2006
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania
// ze wzgledu na drugi proces korzystajacy z tego samego przerwania - tasmociag
// ------------------------------------------------------------------------

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#if defined(linux)
#include <sys/io.h>
#elif defined(__FreeBSD__)
#include <machine/cpufunc.h>
#endif

#include <boost/shared_ptr.hpp>

#include "base/lib/periodic_timer.h"
#include "robot/hi_rydz/hi_rydz.h"
#include "base/edp/edp_e_motor_driven.h"

namespace mrrocpp {
namespace edp {
namespace hi_rydz {

void HI_rydz::init()
{
	// Sledzenie zera rezolwera - wylaczane
	trace_resolver_zero = false;

	irq_data.md.is_power_on = true;
	irq_data.md.is_robot_blocked = false;
	irq_data.md.hardware_error = 0;

	if (master.robot_test_mode) {
		// domyslnie robot jest zsynchronizowany
		irq_data.md.is_synchronised = true;

		ptimer = (boost::shared_ptr<lib::periodic_timer>) new lib::periodic_timer(1000 * lib::EDP_STEP);
	} else {
		// domyslnie robot nie jest zsynchronizowany
		irq_data.md.is_synchronised = false;

#if defined(linux)
		// grant I/O port permissions to the first card region
		if (ioperm(hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset, 0xC, 1) == -1) {
			perror("ioperm()");
		}

		// grant I/O port permissions to the second card region
		if (ioperm(hi_rydz::ADR_OF_SERVO_PTR + hi_isa_card_offset, 1, 1) == -1) {
			perror("ioperm()");
		}
#endif

		irq_data.md.interrupt_mode = common::INT_EMPTY;

		// konieczne dla skasowania przyczyny przerwania
		out8((hi_rydz::ADR_OF_SERVO_PTR + hi_isa_card_offset), hi_intr_generator_servo_ptr);
		in16((hi_rydz::SERVO_REPLY_STATUS_ADR + hi_isa_card_offset)); // Odczyt stanu wylacznikow
		in16((hi_rydz::SERVO_REPLY_INT_ADR + hi_isa_card_offset));


	}

	// oczekiwanie na przerwanie
	if (hi_int_wait(common::INT_EMPTY, 0) == -1) // jesli nie przyjdzie na czas
	{
		// inicjacja wystawiania przerwan
		if (!master.robot_test_mode) {
			// Ustawienie czestotliwosci przerwan
			uint16_t int_freq = SET_INT_FREQUENCY | hi_intr_freq_divider;
			out8((hi_rydz::ADR_OF_SERVO_PTR + hi_isa_card_offset), hi_intr_generator_servo_ptr);
			out16((hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset), int_freq);
			delay(10);
			out16((hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset), START_CLOCK_INTERRUPTS);
		}
	}

	master.controller_state_edp_buf.is_synchronised = irq_data.md.is_synchronised;

	for (int i = 0; i < master.number_of_servos; i++) {
		robot_status[i].adr_offset_plus_0 = 0;
		robot_status[i].adr_offset_plus_2 = 0;
		robot_status[i].adr_offset_plus_4 = 0;
		robot_status[i].adr_offset_plus_6 = 0;
		measured_current[i] = 0;
	}

	// Zakaz pracy recznej we wszystkich osiach

	if (!master.robot_test_mode) {
		for (int i = 0; i < master.number_of_servos; i++) {
			/*
			 out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			 out16((hi_rydz::SERVO_COMMAND1_ADR + ISA_CARD_OFFSET),RESET_MANUAL_MODE); // Zerowanie ruchow recznych
			 out16((hi_rydz::SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), PROHIBIT_MANUAL_MODE); // Zabrania ruchow za pomoca przyciskow w szafie
			 */
			irq_data.md.card_adress = hi_first_servo_ptr + (uint8_t) i;
			irq_data.md.register_adress = (hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset);
			irq_data.md.value = RESET_MANUAL_MODE;
			hi_int_wait(common::INT_SINGLE_COMMAND, 2);
			irq_data.md.value = PROHIBIT_MANUAL_MODE;
			hi_int_wait(common::INT_SINGLE_COMMAND, 2);
			irq_data.md.value = max_current[i];
			hi_int_wait(common::INT_SINGLE_COMMAND, 2);

		}
		// Zerowanie licznikow polozenia wszystkich osi
		reset_counters();
		is_hardware_error();
	}
}

// Konstruktor
HI_rydz::HI_rydz(common::motor_driven_effector &_master, int _hi_irq_real, unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high, unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, const int _max_current[]) :
	HardwareInterface(_master), hi_irq_real(_hi_irq_real), hi_intr_freq_divider(_hi_intr_freq_divider),
			hi_intr_timeout_high(_hi_intr_timeout_high), hi_first_servo_ptr(_hi_first_servo_ptr),
			hi_isa_card_offset(_hi_isa_card_offset), hi_intr_generator_servo_ptr(_hi_intr_generator_servo_ptr)
{
	for (int i = 0; i < master.number_of_servos; i++) {
		max_current[i] = _max_current[i];
	}
}

void HI_rydz::insert_set_value(int drive_number, double set_value)
{ // Wprowadzenie wartosci zadanej PWM
	robot_control[drive_number].adr_offset_plus_0 = (uint16_t) fabs(set_value);
	if (set_value < 0)
		robot_control[drive_number].adr_offset_plus_0 |= 0x300;
	else
		robot_control[drive_number].adr_offset_plus_0 |= 0x200;
}

// dla wybranej osi
int HI_rydz::get_current(int drive_number)
{ // Pobranie pradu
	return measured_current[drive_number];
}

// Pobranie przyrostu polozenia wybranej osi
double HI_rydz::get_increment(int drive_number)
{
	return current_position_inc[drive_number];
}

// Pobranie polozenia wybranej osi
long int HI_rydz::get_position(int drive_number)
{
	return current_absolute_position[drive_number];
}

// ------------------------------------------------------------------------
HI_rydz::~HI_rydz(void) // destruktor
{
	if (!master.robot_test_mode) {
		reset_counters();

		// Zezwolenie na prace reczna
		for (int i = 0; i < master.number_of_servos; i++) {
			irq_data.md.card_adress = hi_first_servo_ptr + (uint8_t) i;
			irq_data.md.register_adress = (hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset);
			irq_data.md.value = ALLOW_MANUAL_MODE;
			hi_int_wait(common::INT_SINGLE_COMMAND, 2);
		}

		// TODO: InterruptDetach(), munmap_device_io()
	}
} // end: hardware_interface::~hardware_interface()
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
uint64_t HI_rydz::read_write_hardware(void)
{
	// ------------------------------------------------------------------------
	// Obsluga sprzetu: odczyt aktualnych wartosci polozenia i zapis wartosci
	// wypelnienia PWM

	// zapis wartosci zadanych
	for (int i = 0; i < master.number_of_servos; i++) {
		irq_data.md.robot_control[i].adr_offset_plus_0 = robot_control[i].adr_offset_plus_0;
	}

	// oczekiwanie na przerwanie
	hi_int_wait(common::INT_SERVOING, 0);

	if (master.robot_test_mode) {
		// Tylko dla testow
		return irq_data.md.hardware_error;
	}

	//	 printf("hi rydz 1 current_absolute_position: %d, hex: %x\n", irq_data.md.current_absolute_position[5], irq_data.md.current_absolute_position[5] ); // debug

	for (int i = 0; i < master.number_of_servos; i++) {

		// przepisanie wartosci pradu
		measured_current[i] = (irq_data.md.robot_status[i].adr_offset_plus_2 & 0xFF00) >> 8;

		current_absolute_position[i] = irq_data.md.current_absolute_position[i];
		current_position_inc[i] = current_absolute_position[i] - previous_absolute_position[i];
		previous_absolute_position[i] = current_absolute_position[i];
	}

	if (!trace_resolver_zero) {
		//	printf("read_write_hardware: w mask resolver_zero\n");
		irq_data.md.hardware_error &= MASK_RESOLVER_ZERO;
	}

	return irq_data.md.hardware_error;
} // end: hardware_interface::read_write_hardware()
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
// Zerowanie licznikow polozenia wszystkich osi
void HI_rydz::reset_counters(void)
{
	for (int i = 0; i < master.number_of_servos; i++) {
		irq_data.md.card_adress = hi_first_servo_ptr + (uint8_t) i;
		irq_data.md.register_adress = (hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset);
		irq_data.md.value = hi_rydz::MICROCONTROLLER_MODE;
		hi_int_wait(common::INT_SINGLE_COMMAND, 2);
		irq_data.md.value = hi_rydz::STOP_MOTORS;
		hi_int_wait(common::INT_SINGLE_COMMAND, 2);
		irq_data.md.value = RESET_MANUAL_MODE;
		hi_int_wait(common::INT_SINGLE_COMMAND, 2);
		irq_data.md.value = RESET_ALARM;
		hi_int_wait(common::INT_SINGLE_COMMAND, 2);

		if (!irq_data.md.is_synchronised) {
			irq_data.md.value = RESET_POSITION_COUNTER;
			hi_int_wait(common::INT_SINGLE_COMMAND, 2);
		}

		current_absolute_position[i] = 0;
		previous_absolute_position[i] = 0;
		current_position_inc[i] = 0.0;

		// 	in16((hi_rydz::SERVO_REPLY_INT_ADR + hi_isa_card_offset));

	} // end: for

	// Dwukrotny odczyt polozenia dla wyzerowania przyrostu wynikajacego z pierwszego
	// odczytu rezolwera
	// wyzerowanie wypelnienia
	for (int i = 0; i < master.number_of_servos; i++) {
		robot_control[i].adr_offset_plus_0 = 0x0200;
	} // end: for

	// wyzerowanie przyrostu pozycji
	read_write_hardware();
	read_write_hardware();
	read_write_hardware();
	// Odczyt polozenia osi slowo 32 bitowe - negacja licznikow 16-bitowych
	// out8((hi_rydz::ADR_OF_SERVO_PTR + hi_isa_card_offset), hi_first_servo_ptr);
	// out16((hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset), RESET_POSITION_COUNTER);
	// robot_status[0].adr_offset_plus_4 = 0xFFFF ^ in16((hi_rydz::SERVO_REPLY_POS_LOW_ADR + hi_isa_card_offset)); // Mlodsze slowo 16-bitowe
	// robot_status[0].adr_offset_plus_6 = 0xFFFF ^ in16((hi_rydz::SERVO_REPLY_POS_HIGH_ADR+ hi_isa_card_offset));// Starsze slowo 16-bitowe
	// printf("L=%x U=%x  \n",robot_status[0].adr_offset_plus_4, robot_status[0].adr_offset_plus_6);
} // end: hardware_interface::reset_counters()
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
bool HI_rydz::is_hardware_error(void)
{
	bool h_error = false;

	// oczekiwanie na przerwanie
	hi_int_wait(common::INT_SINGLE_COMMAND, 0);

	for (int i = 0; i < master.number_of_servos; i++) {
		uint16_t MASK = 0x7E00;

		if ((irq_data.md.robot_status[i].adr_offset_plus_0 ^ 0x6000) & MASK) {
			h_error = true;
			//    printf(" \n => axis= %d r210H: %x ",i,robot_status[i].adr_offset_plus_0);
		}
	} // end: for
	return h_error;
} // end: hardware_interface::is_hardware_error ()
// ------------------------------------------------------------------------


int HI_rydz::hi_int_wait(common::interrupt_mode_t _interrupt_mode, int lag)
{
	if (!master.robot_test_mode) {

		return -1;

	} else {
		ptimer->sleep();

		return 0;
	}
}

void HI_rydz::start_synchro(int drive_number)
{
	trace_resolver_zero = true;
	// Wlacz sledzenie zera rezolwera (synchronizacja robota)
	irq_data.md.card_adress = hi_first_servo_ptr + (uint8_t) drive_number;
	irq_data.md.register_adress = (hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset);
	irq_data.md.value = START_SYNCHRO;
	hi_int_wait(common::INT_SINGLE_COMMAND, 2);
} // end: start_synchro()

void HI_rydz::finish_synchro(int drive_number)
{
	trace_resolver_zero = false;

	// Zakonczyc sledzenie zera rezolwera i przejdz do trybu normalnej pracy
	irq_data.md.card_adress = hi_first_servo_ptr + (uint8_t) drive_number;
	irq_data.md.register_adress = (hi_rydz::SERVO_COMMAND1_ADR + hi_isa_card_offset);
	irq_data.md.value = hi_rydz::FINISH_SYNCHRO;
	hi_int_wait(common::INT_SINGLE_COMMAND, 2);

	// by Y - UWAGA NIE WIEDZIEC CZEMU BEZ TEGO NIE ZAWSZE DZIALAJA RUCHY NA OSI PO SYNCHRONIZACJi
	irq_data.md.value = hi_rydz::MICROCONTROLLER_MODE;
	hi_int_wait(common::INT_SINGLE_COMMAND, 2);
} // end: finish_synchro()


bool HI_rydz::in_synchro_area(int drive_number)
{
	return false;
}

bool HI_rydz::robot_synchronized()
{
	return false;
}

// Sprawdzenie czy pojawilo sie zero  (synchronizacji rezolwera)
bool HI_rydz::is_impulse_zero(int drive_number)
{
	if (robot_status[drive_number].adr_offset_plus_0 & 0x0100)
		return true;
	else
		return false;
}

// Zerowanie licznikow polozenia
void HI_rydz::reset_position(int i)
{
	current_absolute_position[i] = 0L;
	previous_absolute_position[i] = 0L;
	current_position_inc[i] = 0.0;
}

int HI_rydz::set_parameter(int drive_number, const int parameter, uint32_t new_value)
{
	return 0;
}

} // namespace hi_rydz
} // namespace edp
} // namespace mrrocpp

