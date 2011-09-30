// ------------------------------------------------------------------------
//                            hi_rydz.cc
//
// Funkcje do obslugi sprzetu (serwomechanizmow cyfrowych) dla robota irp6 on_track
//
// cala komunikacja ze sprzetem przerzucona do oblsugi przerwania ze wzgledu na drugi proces korzystajacy z tego samego
// przerwania - tasmociag
// ------------------------------------------------------------------------

// Klasa edp_irp6ot_effector.
#include "robot/irp6ot_m/edp_irp6ot_m_effector.h"
// Klasa hardware_interface.
#include "robot/irp6ot_m/hi_irp6ot_m.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_m {

// ------------------------------------------------------------------------
hardware_interface::hardware_interface(common::motor_driven_effector &_master, int _hi_irq_real, unsigned short int _hi_intr_freq_divider, unsigned int _hi_intr_timeout_high, unsigned int _hi_first_servo_ptr, unsigned int _hi_intr_generator_servo_ptr, unsigned int _hi_isa_card_offset, const int _max_current[]) :
			hi_rydz::HI_rydz(_master, _hi_irq_real, _hi_intr_freq_divider, _hi_intr_timeout_high, _hi_first_servo_ptr, _hi_intr_generator_servo_ptr, _hi_isa_card_offset, _max_current)
{
}
// ------------------------------------------------------------------------


// ------------------------------------------------------------------------
uint64_t hardware_interface::read_write_hardware(void)
{
	// ------------------------------------------------------------------------
	// Obsluga sprzetu: odczyt aktualnych wartosci polozenia i zapis wartosci
	// wypelnienia PWM

	// zapis wartosci zadanych
	for (int i = 0; i < master.number_of_servos; i++) {
		irq_data.md.robot_control[i].adr_offset_plus_0 = robot_control[i].adr_offset_plus_0;
	}

	// oczekiwanie na przerwanie
	hi_int_wait(edp::common::INT_SERVOING, 0);

	if (master.robot_test_mode) {
		// Tylko dla testow
		return irq_data.md.hardware_error;
	}

	//	 printf("hi rydz 1 current_absolute_position: %d, hex: %x\n", irq_data.md.current_absolute_position[5], irq_data.md.current_absolute_position[5] ); // debug

	for (int i = 0; i < master.number_of_servos; i++) {

		// przepisanie wartosci pradu
		if (i < 6) // osie rezolwerowe
		{
			measured_current[i] = irq_data.md.robot_status[i].adr_offset_plus_0 & 0x00FF;
		} else {
			measured_current[i] = (irq_data.md.robot_status[i].adr_offset_plus_2 & 0xFF00) >> 8;
		}

		current_absolute_position[i] = irq_data.md.current_absolute_position[i];
		current_position_inc[i] = current_absolute_position[i] - previous_absolute_position[i];
		previous_absolute_position[i] = current_absolute_position[i];
	}

	if (!trace_resolver_zero) {
		//	printf("read_write_hardware: w mask resolver_zero\n");
		irq_data.md.hardware_error &= hi_rydz::MASK_RESOLVER_ZERO;
	}

	return irq_data.md.hardware_error;
} // end: hardware_interface::read_write_hardware()
// ------------------------------------------------------------------------


void hardware_interface::finish_synchro(int drive_number)
{
	trace_resolver_zero = false;

	// Zakonczyc sledzenie zera rezolwera i przejdz do trybu normalnej pracy
	irq_data.md.card_adress = FIRST_SERVO_PTR + (uint8_t) drive_number;
	irq_data.md.register_adress = (hi_rydz::SERVO_COMMAND1_ADR + ISA_CARD_OFFSET);
	irq_data.md.value = hi_rydz::FINISH_SYNCHRO;
	hi_int_wait(edp::common::INT_SINGLE_COMMAND, 2);

	// by Y - UWAGA NIE WIEDZIEC CZEMU BEZ TEGO NIE ZAWSZE DZIALAJA RUCHY NA OSI PO SYNCHRONIZACJi
	if (drive_number > 5) {
		irq_data.md.value = hi_rydz::MICROCONTROLLER_MODE;
		hi_int_wait(edp::common::INT_SINGLE_COMMAND, 2);
	}

} // end: finish_synchro()

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
