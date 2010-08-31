// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota conveyor
//
// Ostatnia modyfikacja: 2005
// ------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <cctype>
#include <sys/wait.h>
#include <sys/types.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>
#endif

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

// Klasa edp_conveyor_effector.
#include "robot/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "robot/conveyor/hi_conv.h"
#include "robot/conveyor/const_conveyor.h"

namespace mrrocpp {
namespace edp {
namespace common {

extern conveyor::effector* master;

}

namespace conveyor {

// ------------------------------------------------------------------------
// Obsluga przerwania sprzetowego
#ifdef __QNXNTO__
const struct sigevent *
int_handler (void *arg, int int_id)
{
	common::irq_data_t *irq_data = (common::irq_data_t *) arg;
	common::motor_data & md = irq_data->md;
	struct sigevent & event = irq_data->event;

	common::status_of_a_dof robot_status[lib::conveyor::NUM_OF_SERVOS];
	short int low_word, high_word;

	md.hardware_error = (uint64_t) common::ALL_RIGHT; // Nie ma bledow sprzetowych
	md.is_synchronised = true; // conveyor jest stale zsynchronizowany

	if(common::master->robot_test_mode)
	{
		return (&event);// by Y&W
	}

	// INT_EMPTY obluga pusta
	if (md.interrupt_mode==edp::common::INT_EMPTY)
	{
		// konieczne dla skasowania przyczyny przerwania
		out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR);
		md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16((hi_rydz::SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16((hi_rydz::SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		return (&event);
	}

	// INT_SERVOING tryb regulacji osi
	else if (md.interrupt_mode==edp::common::INT_SERVOING)
	{
		// Odczyty stanu osi, polozenia oraz pradu wirnikow
		out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR);
		md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16((hi_rydz::SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow

		md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16((hi_rydz::SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		// Odczyt polozenia osi: slowo 32 bitowe - negacja licznikow 16-bitowych
		robot_status[0].adr_offset_plus_4 = 0xFFFF ^ in16((hi_rydz::SERVO_REPLY_POS_LOW_ADR + ISA_CARD_OFFSET)); // Mlodsze slowo 16-bitowe
		robot_status[0].adr_offset_plus_6 = 0xFFFF ^ in16((hi_rydz::SERVO_REPLY_POS_HIGH_ADR+ ISA_CARD_OFFSET)); // Starsze slowo 16-bitowe

		md.robot_status[0].adr_offset_plus_4 = robot_status[0].adr_offset_plus_4;
		md.robot_status[0].adr_offset_plus_6 = robot_status[0].adr_offset_plus_6;

		low_word = robot_status[0].adr_offset_plus_4;
		high_word = robot_status[0].adr_offset_plus_6;

		// Obliczenie polozenia
		md.current_absolute_position[0] = (((uint32_t) (high_word<<16)) & (0xFFFF0000)) | ((uint16_t) low_word);

		//   md.robot_status[i].adr_offset_plus_6 = robot_status[i].adr_offset_plus_6;
		//   md.high_word = high_word;

		//  md.robot_status[i].adr_offset_plus_8 = robot_status[i].adr_offset_plus_8 = in16((hi_rydz::SERVO_REPLY_REG_1_ADR + ISA_CARD_OFFSET)); // Niewykorzystane
		//  md.robot_status[i].adr_offset_plus_a = robot_status[i].adr_offset_plus_a = in16((hi_rydz::SERVO_REPLY_REG_2_ADR + ISA_CARD_OFFSET)); // Niewykorzystane

		// Obsluga bledow

		if ( robot_status[0].adr_offset_plus_0 & 0x0100 )
		md.hardware_error |= (uint64_t) (common::SYNCHRO_ZERO); // Impuls zera rezolwera

		if ( robot_status[0].adr_offset_plus_0 & 0x4000 )
		md.hardware_error |= (uint64_t) (common::SYNCHRO_SWITCH_ON); // Zadzialal wylacznik synchronizacji


		if ( robot_status[0].adr_offset_plus_0 & 0x0400 )
		{
			md.hardware_error |= (uint64_t) (common::OVER_CURRENT);
			//     out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			//     out16((hi_rydz::SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
		}

		if (robot_status[0].adr_offset_plus_0 & 0x0080) // czy wlaczono moc
		{
			md.is_power_on = true;
		} else
		{
			md.is_power_on = false;
			md.is_robot_blocked = true;
		}

		if ( md.hardware_error & hi_rydz::HARDWARE_ERROR_MASK ) // wyciecie SYNCHRO_ZERO i SYNCHRO_SWITCH_ON
		{

			// Zapis wartosci zadanej wypelnienia PWM
			out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR);
			out16((hi_rydz::SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), hi_rydz::STOP_MOTORS);

			return (&event); // Yoyek & 7
		}

		// Zapis wartosci zadanej wypelnienia PWM
		out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR);
		if (md.is_robot_blocked)
		md.robot_control[0].adr_offset_plus_0 &= 0xff00;
		out16((hi_rydz::SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), md.robot_control[0].adr_offset_plus_0);

		return (&event);
	} // end INT_SERVOING

	// INT_SINGLE_COMMAND do synchronizacji, inicjacji, etc.
	else if (md.interrupt_mode==edp::common::INT_SINGLE_COMMAND)
	{
		out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), md.card_adress);
		out16(md .register_adress, md.value);
		// konieczne dla skasowania przyczyny przerwania

		out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR);
		md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16((hi_rydz::SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16((hi_rydz::SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		md.interrupt_mode=edp::common:: INT_EMPTY; // aby tylko raz wyslac polecenie
		return (&event);
	}

	// INT_CHECK_STATE do odczytu stanu z adresu 0x220
	else if (md.interrupt_mode==edp::common::INT_CHECK_STATE)
	{
		// konieczne dla skasowania przyczyny przerwania
		out8((hi_rydz::ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR);
		md.robot_status[0].adr_offset_plus_0 = robot_status[0].adr_offset_plus_0 = in16((hi_rydz::SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		md.robot_status[0].adr_offset_plus_2 = robot_status[0].adr_offset_plus_2 = in16((hi_rydz::SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		md.interrupt_mode=edp::common::INT_EMPTY; // aby tylko raz sprawdzic stan
		return (&event);
	}

	// Zakonczenie obslugi przerwania ze wzbudzeniem posrednika (proxy)

	return (&event);// Yoyek & wojtek

}
#endif /*__QNXNTO__ */

// ------------------------------------------------------------------------

} // namespace conveyor
} // namespace edp
} // namespace mrrocpp
