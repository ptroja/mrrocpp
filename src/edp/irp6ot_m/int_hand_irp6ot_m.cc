// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota irp6 on_track
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/types.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_irp6ot_effector.
#include "edp/irp6ot_m/edp_irp6ot_m_effector.h"
// Klasa hardware_interface.
#include "edp/irp6ot_m/hi_irp6ot_m.h"
#include "edp/common/in_out.h"

namespace mrrocpp {
namespace edp {
namespace common {

extern irp6ot_m::effector* master; // Bufor polecen i odpowiedzi EDP_MASTER

}

namespace irp6ot_m {

// ------------------------------------------------------------------------

// Obsluga przerwania sprzetowego

// UWAGA - zmienna ilosc serwomechanizmow w zaleznosci od tego czy gripper jest dolaczony czy nie
#ifdef __QNXNTO__
const struct sigevent *
int_handler (void *arg, int int_id)
{
	common::irq_data_t *irq_data = (common::irq_data_t *) arg;
	common::motor_data & md = irq_data->md;
	struct sigevent & event = irq_data->event;

	common::status_of_a_dof robot_status[IRP6OT_M_NUM_OF_SERVOS];
	short int low_word, high_word;

	md.hardware_error = (uint64_t) lib::ALL_RIGHT; // Nie ma bledow sprzetowych

	if(common::master->test_mode)
	{
		return (&event); // by Y&W
	}

	// INT_EMPTY obluga pusta
	// z zalozenia to pierwszy tryb w ktorym jest uruchomiona fukcja obslugi przewania  ze wzgledu na synchronizacje
	if (md.interrupt_mode == edp::common::INT_EMPTY)
	{
		// konieczne dla skasowania przyczyny przerwania
		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), INTERRUPT_GENERATOR_SERVO_PTR);
		in16((SERVO_REPLY_STATUS_ADR + ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		md.is_synchronised = true;
		for (int i = 0; i < common::master->number_of_servos; i++ )
		{
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
			md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

			if (i>5) // dla osi mechatronicznych
			{
				// jesli ktorakolwiek os jest niezsynchronizwana to i robot jest niezsynchronizowany
				if (!(robot_status[i].adr_offset_plus_0 & 0x0040))
				{
					md.is_synchronised = false;
				}
			}
		}

		return (&event);
	}

	// INT_SERVOING tryb regulacji osi
	else if (md.interrupt_mode == edp::common::INT_SERVOING)
	{

		// konieczne dla skasowania przyczyny przerwania
		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), INTERRUPT_GENERATOR_SERVO_PTR);
		in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		for (int i = 0; i < common::master->number_of_servos; i++ )
		{
			// Odczyty stanu osi, polozenia oraz pradu wirnikow
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow

			md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

			// Odczyt polozenia osi: slowo 32 bitowe - negacja licznikow 16-bitowych
			robot_status[i].adr_offset_plus_4 = 0xFFFF ^ in16((SERVO_REPLY_POS_LOW_ADR + ISA_CARD_OFFSET)); // Mlodsze slowo 16-bitowe
			robot_status[i].adr_offset_plus_6 = 0xFFFF ^ in16((SERVO_REPLY_POS_HIGH_ADR+ ISA_CARD_OFFSET));// Starsze slowo 16-bitowe

			if (i<6) // osie rezolwerowe
			{
				// Wycinanie niewykorzystywanych bitow mlodszego slowa licznika polozenia
				if (robot_status[i].adr_offset_plus_4 & 0x0800)
				// Zly odczyt polozenia
				robot_status[i].adr_offset_plus_4 &= 0x0000;
				else
				robot_status[i].adr_offset_plus_4 &= 0x07FF;
			}

			md.robot_status[i].adr_offset_plus_4 = robot_status[i].adr_offset_plus_4;
			md.robot_status[i].adr_offset_plus_6 = robot_status[i].adr_offset_plus_6;

			low_word = robot_status[i].adr_offset_plus_4;
			high_word = robot_status[i].adr_offset_plus_6;

			if (i<6) // osie z rezolwerami
			{
				md.current_absolute_position[i] = ((uint32_t) (high_word* (int)(IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION))) + ((uint32_t) low_word);
			} else
			{ // osie z enkoderami
				md.current_absolute_position[i] = (((uint32_t) (high_word<<16)) & (0xFFFF0000)) | ((uint16_t) low_word);
			}

			//   md.robot_status[i].adr_offset_plus_6 = robot_status[i].adr_offset_plus_6;
			//   md.high_word = high_word;

			//  md.robot_status[i].adr_offset_plus_8 = robot_status[i].adr_offset_plus_8 = in16((SERVO_REPLY_REG_1_ADR + ISA_CARD_OFFSET)); // Niewykorzystane
			//  md.robot_status[i].adr_offset_plus_a = robot_status[i].adr_offset_plus_a = in16((SERVO_REPLY_REG_2_ADR + ISA_CARD_OFFSET)); // Niewykorzystane


			// Obsluga bledow
			if ( robot_status[i].adr_offset_plus_0 & 0x0100 )
			md.hardware_error |= (uint64_t) (lib::SYNCHRO_ZERO << (5*i)); // Impuls zera rezolwera

			if (i>=6) //  sterowniki osi z mechatroniki z przekaznikami
			{
				if ( ~(robot_status[i].adr_offset_plus_0) & 0x4000 )
				md.hardware_error |= (uint64_t) (lib::SYNCHRO_SWITCH_ON << (5*i)); // Zadzialal wylacznik synchronizacji
			} else
			{
				if ( robot_status[i].adr_offset_plus_0 & 0x8000 )
				md.hardware_error |= (uint64_t) (lib::SYNCHRO_SWITCH_ON << (5*i)); // Zadzialal wylacznik synchronizacji
			}

			// wylaczniki krancowe
			if (i>=6) //  sterowniki osi z mechatroniki z przekaznikami
			{
				if ( ~(robot_status[i].adr_offset_plus_0) & 0x1000 )
				{
					//	out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
					//	out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
					md.hardware_error |= (uint64_t) (lib::UPPER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "gorny" krancowy
				}
				else if ( ~(robot_status[i].adr_offset_plus_0) & 0x2000 )
				{
					md.hardware_error |= (uint64_t) (lib::LOWER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "dolny" krancowy
				}
			} else
			{
				if ( ~(robot_status[i].adr_offset_plus_0) & 0x2000 )
				{
					//	out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
					//	out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
					md.hardware_error |= (uint64_t) (lib::UPPER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "gorny" krancowy
				}
				else if ( ~(robot_status[i].adr_offset_plus_0) & 0x4000 )
				{
					md.hardware_error |= (uint64_t) (lib::LOWER_LIMIT_SWITCH << (5*i)); // Zadzialal wylacznik "dolny" krancowy
				}
			}

			if ( robot_status[i].adr_offset_plus_0 & 0x0400 )
			{
				md.hardware_error |= (uint64_t) (lib::OVER_CURRENT << (5*i));
				//     out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
				//     out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), RESET_ALARM); // Skasowanie alarmu i umozliwienie ruchu osi
			}
		}

		if (robot_status[6].adr_offset_plus_0 & 0x0080) // czy wlaczono moc
		{
			md.is_power_on = true;
		} else
		{
			md.is_robot_blocked = true;
			md.is_power_on = false;
		}

		if ( md.hardware_error & lib::HARDWARE_ERROR_MASK ) // wyciecie SYNCHRO_ZERO i SYNCHRO_SWITCH_ON
		{
			for (int i = 0; i < common::master->number_of_servos; i++ )
			{
				// Zapis wartosci zadanej wypelnienia PWM
				out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
				out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), STOP_MOTORS);
			}
			return (&event); // Yoyek & 7
		}

		for (int i = 0; i < common::master->number_of_servos; i++ )
		{
			// Zapis wartosci zadanej wypelnienia PWM
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			if (md.is_robot_blocked)
			md.robot_control[i].adr_offset_plus_0 &= 0xff00;
			out16((SERVO_COMMAND1_ADR + ISA_CARD_OFFSET), md.robot_control[i].adr_offset_plus_0);
		}

		uint16_t binary_input, binary_output;
		uint8_t analog_input[8];
		uint16_t tmp_buf; // do uzyku przy odczcie wejsc

		// odczyt wejsc analogowych (z przetwornikow) i binarnych
		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), IN_OUT_PACKET);

		binary_input = in16(SERVO_REPLY_REG_1_ADR + ISA_CARD_OFFSET);

		tmp_buf=in16(SERVO_REPLY_STATUS_ADR + ISA_CARD_OFFSET);
		analog_input[0]=0x00ff & tmp_buf;
		analog_input[1]=((0xff00 & tmp_buf)>>8);

		tmp_buf=in16(SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET);
		analog_input[2]=0x00ff & tmp_buf;
		analog_input[3]=((0xff00 & tmp_buf)>>8);

		tmp_buf=in16(SERVO_REPLY_POS_LOW_ADR + ISA_CARD_OFFSET);
		analog_input[4]=0x00ff & tmp_buf;
		analog_input[5]=((0xff00 & tmp_buf)>>8);

		tmp_buf=in16(SERVO_REPLY_POS_HIGH_ADR + ISA_CARD_OFFSET);
		analog_input[6]=0x00ff & tmp_buf;
		analog_input[7]=((0xff00 & tmp_buf)>>8);

		common::master->in_out_obj->set_input(binary_input, analog_input);

		// ustawienie wyjscia o ile bylo takie zlecenie
		if (common::master->in_out_obj->set_output_flag)
		{
			common::master->in_out_obj->set_output_flag=false;
			common::master->in_out_obj->get_output(&binary_output);

			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), IN_OUT_PACKET);
			// (SERVO_COMMAND2_ADR + ISA_CARD_OFFSET)       0x212
			out16((SERVO_COMMAND2_ADR + ISA_CARD_OFFSET), binary_output);
		}

		return (&event);
	} // end INT_SERVOING

	// INT_SINGLE_COMMAND do synchronizacji, inicjacji, etc.
	else if (md.interrupt_mode == edp::common::INT_SINGLE_COMMAND)
	{

		// konieczne dla skasowania przyczyny przerwania
		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), INTERRUPT_GENERATOR_SERVO_PTR);
		in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), md.card_adress);
		out16(md .register_adress, md.value);

		for (int i = 0; i < common::master->number_of_servos; i++ )
		{
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
			md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));
		}
		md.interrupt_mode=edp::common::INT_EMPTY; // aby tylko raz wyslac polecenie

		return (&event);
	}

	// INT_CHECK_STATE do odczytu stanu z adresu 0x220
	else if (md.interrupt_mode == edp::common::INT_CHECK_STATE)
	{

		// konieczne dla skasowania przyczyny przerwania
		out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), INTERRUPT_GENERATOR_SERVO_PTR);
		in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
		in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));

		for (int i = 0; i < common::master->number_of_servos; i++ )
		{
			out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), FIRST_SERVO_PTR + (uint8_t)i);
			md.robot_status[i].adr_offset_plus_0 = robot_status[i].adr_offset_plus_0 = in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)); // Odczyt stanu wylacznikow
			md.robot_status[i].adr_offset_plus_2 = robot_status[i].adr_offset_plus_2 = in16((SERVO_REPLY_INT_ADR + ISA_CARD_OFFSET));
		}
		md.interrupt_mode=edp::common::INT_EMPTY; // aby tylko raz sprawdzic stan

		return (&event);
	}

	// Zakonczenie obslugi przerwania ze wzbudzeniem posrednika (proxy)

	return (&event);// Yoyek & wojtek
}
#endif /*__QNXNTO__ */

// ------------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp

