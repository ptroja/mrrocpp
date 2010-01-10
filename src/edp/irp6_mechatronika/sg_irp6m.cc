/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_irp6m_effector.
#include "edp/irp6_mechatronika/edp_irp6m_effector.h"
#include "edp/common/reader.h"
// Klasa hardware_interface.
#include "edp/irp6_mechatronika/hi_irp6m.h"
// Klasa servo_buffer.
#include "edp/irp6_mechatronika/sg_irp6m.h"
#include "edp/irp6_mechatronika/regulator_irp6m.h"

namespace mrrocpp {
namespace edp {
namespace irp6m {

/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::Move_a_step(void)
{
	// wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH oraz SYNCHRO_ZERO

	Move_1_step();
	if (master.is_synchronised()) {// by Y aktualizacja transformera am jedynie sens po synchronizacji (kiedy robot zna swoja pozycje)
		// by Y - do dokonczenia
		for (int i = 0; i < master.number_of_servos; i++) {
			if (!(master.test_mode)) {
				master.update_servo_current_motor_pos_abs(hi->get_position(i) * (2*M_PI) / axe_inc_per_revolution[i], i);
			}
		}

		master.servo_joints_and_frame_actualization_and_upload();// by Y - aktualizacja trasformatora
	}
	return convert_error();
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{

	for (int j = 0; j < master.number_of_servos; j++) {
		axe_inc_per_revolution[j] = IRP6_MECHATRONIKA_AXIS_0_TO_5_INC_PER_REVOLUTION;
	}

	thread_id = new boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{

	// tablica pradow maksymalnych d;a poszczegolnych osi
	int
			max_current[IRP6_MECHATRONIKA_NUM_OF_SERVOS] = { IRP6_MECHATRONIKA_AXIS_1_MAX_CURRENT, IRP6_MECHATRONIKA_AXIS_2_MAX_CURRENT, IRP6_MECHATRONIKA_AXIS_3_MAX_CURRENT, IRP6_MECHATRONIKA_AXIS_4_MAX_CURRENT, IRP6_MECHATRONIKA_AXIS_5_MAX_CURRENT };

	hi
			= new hardware_interface(master, IRQ_REAL, INT_FREC_DIVIDER, HI_RYDZ_INTR_TIMEOUT_HIGH, FIRST_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR, ISA_CARD_OFFSET, max_current);
	hi->init();

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1

	// regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
	// kolumna dla irp6 mechatronika
	regulator_ptr[0] = new NL_regulator_2_irp6m(0, 0, 0.429, 6.834, 6.606, 0.35, master); // kolumna dla irp6 mechatronika

	regulator_ptr[1] = new NL_regulator_3_irp6m(0, 0, 0.64, 9.96 / 4, 9.54 / 4, 0.35, master);

	// regulator_ptr[3] = new NL_regulator_4 (0, 0, 0.62, 9.85/4, 9.39/4, 0.35);
	regulator_ptr[2] = new NL_regulator_4_irp6m(0, 0, 0.333, 5.693, 5.427, 0.35, master);

	regulator_ptr[3] = new NL_regulator_5_irp6m(0, 0, 0.56, 7.98 / 2, 7.55 / 2, 0.35, master);

	// regulator_ptr[5] = new NL_regulator_6 (0, 0, 0.3079*2, 0.6, 0.6, 0.35);
	regulator_ptr[4] = new NL_regulator_6_irp6m(0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);
	// regulator_ptr[0] = new NL_regulator_1 (0, 0, 0.64, 16.61/5., 15.89/5., 0.35);


	send_after_last_step = false;
	clear_reply_status();
	clear_reply_status_tmp();

	for (int j = 0; j < master.number_of_servos; j++) {
		command.parameters.move.abs_position[j] = 0.0;
	}

}

/*-----------------------------------------------------------------------*/

void servo_buffer::synchronise(void)
{

	const int NS = 10; // liczba krokow rozpedzania/hamowania
	common::regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

	double synchro_step = 0.0; // zadany przyrost polozenia

	if (master.test_mode) {
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();
		reply_to_EDP_MASTER();
		return;
	}

	for (int j = 0; j < (master.number_of_servos); j++) {
		command.parameters.move.abs_position[j] = 0.0;
	}

	// szeregowa synchronizacja serwomechanizmow
	for (int k = 0; k < (master.number_of_servos); k++) {
		int j = ((k + IRP6M_SYN_INIT_AXE) % (master.number_of_servos));

		// printf("os synchronizopwana: %d \n",j);
		for (int l = 0; l < (master.number_of_servos); l++) {
			int i = ((l + IRP6M_SYN_INIT_AXE) % (master.number_of_servos));
			// zerowy przyrost polozenia dla wszystkich napedow procz j-tego
			if (i == j) {
				crp = regulator_ptr[i];
				// W.S.        crp->insert_new_step(lib::SYNCHRO_STEP_COARSE);

				synchro_step = IRP6_MECHATRONIKA_SYNCHRO_STEP_COARSE / NS;

				crp->insert_new_step(synchro_step);
			} else {
				regulator_ptr[i]->insert_new_step(0.0);
			}
		}

		clear_reply_status();
		clear_reply_status_tmp();

		synchro_step = 0.0;

		// ruch do wykrycia wylacznika synchronizacji
		for (;;) {
			do {

				if (synchro_step > IRP6_MECHATRONIKA_SYNCHRO_STEP_COARSE) {
					synchro_step += IRP6_MECHATRONIKA_SYNCHRO_STEP_COARSE / NS;
					crp->insert_new_step(synchro_step);
				}

				//		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
			} while (Move_1_step() == NO_ERROR_DETECTED); // end: while
			//		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
			// analiza przyslanego bledu (czy wjechano na wylacznik synchronizacji?)
			// jezeli nie, to blad
			switch ((reply_status_tmp.error0 >> (5* j )) & 0x000000000000001FULL)
			{
				case lib::SYNCHRO_SWITCH_ON:
					//  printf("aaa: SYNCHRO_SWITCH_ON\n");
				case lib::SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
					// cprintf("B=%lx\n", reply_status_tmp.error0);
					//		printf("aaa: SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO\n");
					break;
				case lib::ALL_RIGHT:
				case lib::SYNCHRO_ZERO:
					//     printf("aaa: SYNCHRO_ZERO\n");
					continue;
				default:
					//    printf("aaa: default\n");
					// awaria w trakcie synchronizacji
					convert_error();
					reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_SWITCH_EXPECTED;
					reply_status.error1 = reply_status_tmp.error1;
					clear_reply_status_tmp();
					reply_to_EDP_MASTER();
					return;
			}
			break;
		}

		//	printf("przed clear_reply_status \n");

		clear_reply_status();
		clear_reply_status_tmp();

		// zatrzymanie na chwile robota
		synchro_step = 0.0;
		crp->insert_new_step(synchro_step);
		for (int i = 0; i < 250; i++) {
			Move_1_step();
			//  printf("aabb: %d, %x\n", j, reply_status_tmp.error0);
			switch ((reply_status_tmp.error0 >> (5* j )) & 0x000000000000001FULL)
			{
				case lib::SYNCHRO_SWITCH_ON:
				case lib::SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
				case lib::ALL_RIGHT:
				case lib::SYNCHRO_ZERO:
					continue;
				default:
					// awaria w trakcie stania
					convert_error();
					reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_DELAY_ERROR;
					reply_status.error1 = reply_status_tmp.error1;
					clear_reply_status_tmp();
					reply_to_EDP_MASTER();
					return;
			}
		}
		// cprintf("C=%lx\n", reply_status_tmp.error0);

		clear_reply_status();
		clear_reply_status_tmp();

		// zjazd z wylacznika synchronizacji
		// W.S.  crp->insert_new_step(lib::SYNCHRO_STEP_FINE);
		synchro_step = -IRP6_MECHATRONIKA_SYNCHRO_STEP_FINE / NS;

		crp->insert_new_step(synchro_step);

		// wlaczenie sledzenia zera rezolwera (synchronizacja osi)
		hi->start_synchro(j);
		delay(1);
		Move_1_step();
		while (1) {
			//  		printf("babb: %d\n", j);
			Move_1_step();
			// W.S. -----------------------------------------------------
			//	printf("ccc: %d\n", j);

			if (synchro_step < -IRP6_MECHATRONIKA_SYNCHRO_STEP_FINE) {
				synchro_step -= IRP6_MECHATRONIKA_SYNCHRO_STEP_FINE / NS;
				crp->insert_new_step(synchro_step);
			}

			// W.S. -----------------------------------------------------
			//    	   printf("bbbb if: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
			switch ((reply_status_tmp.error0 >> (5* j )) & 0x000000000000001FULL)
			{
				case lib::SYNCHRO_SWITCH_ON:
					//    	printf("bcbb: SYNCHRO_SWITCH_ON\n");
				case lib::SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO:
					//     	printf("bfbb: SYNCHRO_SWITCH_ON_AND_SYNCHRO_ZERO\n");
					continue;
				default:
					//   	printf("baabb: default\n");
					break;
			}
			break;
		}
		//	 printf("D\n ");

		// analiza powstalej sytuacji (czy zjechano z wylacznika synchronizacji)
		// jezeli nie, to blad
		switch (((reply_status_tmp.error0 >> (5* j )) & 0x000000000000001FULL))
		{
			case lib::SYNCHRO_ZERO: // zjechano z wylacznika synchronizacji i SYNCHRO_ZERO jest od razu
				//     printf("SYNCHRO_ZERO\n");
				hi->finish_synchro(j);

				//	 printf("SYNCHRO_ZERO\n");
				hi->reset_position(j);
				crp->clear_regulator();
				delay(1);
				continue;
			case OK:
				// ruch do wykrycia zera rezolwera
				//    printf("OK\n");
				for (;;) {
					Move_1_step();
					//        printf("OK Move_1_step\n");
					//      if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL) != OK)
					// by Y - wyciecie SYNCHRO_SWITCH_ON - ze wzgledu na wystepujace drgania
					if (((reply_status_tmp.error0 >> (5* j )) & 0xCE739CE739CE739DULL) != OK) {
						//   printf("OK os: %d, if: %llx, %llx\n", j, reply_status_tmp.error0, ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL));
						// Usuniecie bitow SYNCHRO_ZERO i SYNCHRO_SWITCH_ON z wszystkich osi
						// oprocz synchronizowanej
						// osie zsynchronizowane nie sa analizowane
						break;
					}
				}
				//     if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) != lib::SYNCHRO_ZERO) {
				// by Y - wyciecie SYNCHRO_SWITCH_ON
				if (((reply_status_tmp.error0 >> (5* j )) & 0x000000000000001DULL) != lib::SYNCHRO_ZERO) {
					// 	  printf("OK convert_error: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
					convert_error();
					reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
					reply_status.error1 = reply_status_tmp.error1;
					clear_reply_status_tmp();
					// Wypelnic servo_data
					reply_to_EDP_MASTER();
					return;
				} else {
					hi->finish_synchro(j);
					hi->reset_position(j);
					crp->clear_regulator();
					delay(1);
					continue;
				}
			default:
				//    	printf(" default error\n");
				// awaria w trakcie synchronizacji
				convert_error();
				reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
				// Wypelnic servo_data
				reply_to_EDP_MASTER();
				return;
		}
		// zakonczenie synchronizacji danej osi i przejscie do trybu normalnego
	}

	// zatrzymanie na chwile robota
	for (int k = 0; k < (master.number_of_servos); k++) {
		int j = ((k + IRP6M_SYN_INIT_AXE) % (master.number_of_servos));
		synchro_step = 0.0;
		crp = regulator_ptr[j];
		crp->insert_new_step(synchro_step);
	}
	for (int i = 0; i < 25; i++)
		Move_1_step();

	// kk = 0;

	// printf("koniec synchro\n");
	reply_to_EDP_MASTER();
	return;
}

/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions(void)
{
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	for (int i = 0; i < master.number_of_servos; i++) {

		servo_data.abs_position[i] = hi->get_position(i) * (2*M_PI) / axe_inc_per_revolution[i];

		// przyrost polozenia w impulsach
		servo_data.position[i] = regulator_ptr[i]->get_position_inc(1);
		servo_data.current[i] = regulator_ptr[i]->get_meassured_current();
		servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
		servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
		servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
	}

}
/*-----------------------------------------------------------------------*/



} // namespace irp6m

namespace common {

servo_buffer* return_created_servo_buffer(manip_and_conv_effector &_master)
{
	return new irp6m::servo_buffer((irp6m::effector &) (_master));
}

} // namespace common
} // namespace edp
} // namespace mrrocpp


