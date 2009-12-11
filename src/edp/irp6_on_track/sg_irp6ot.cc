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

// Klasa edp_irp6ot_effector.
#include "edp/irp6_on_track/edp_irp6ot_effector.h"
#include "edp/common/reader.h"
// Klasa hardware_interface.
#include "edp/irp6_on_track/hi_irp6ot.h"
// Klasa servo_buffer.
#include "edp/irp6_on_track/sg_irp6ot.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot {



// uint64_t kk;				  // numer pomiaru od momentu startu pomiarow

/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::Move_a_step (void)
{
	// wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH ora SYNCHRO_ZERO
	Move_1_step ();
	//	printf("bbb\n");
	// by Y aktualizacja transformera ma jedynie sens po synchronizacji (kiedy robot zna swoja pozycje)
	if (master.is_synchronised())
	{
		for (int i=0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
		{
			if (!(master.test_mode))
			{
				switch (i)
				{
				case IRP6OT_GRIPPER_CATCH_AXE:
					master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION, i);
					break;
				case IRP6OT_GRIPPER_TURN_AXE:
					master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION, i);
					break;
				default:
					master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION, i);
					break;
				}
			}

			/*
            		if (i==7)
        {
            	  master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION,  i);
             } else if (i==6)
        {
            	master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION,  i);
             } else
        {
            	master.update_servo_current_motor_pos(regulator_ptr[i]->get_position_inc(0)*2*M_PI/IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION,  i);
             }
			 */
		}

		//		printf("aallalal\n");
		// by Y - aktualizacja trasformatora
		master.servo_joints_and_frame_actualization_and_upload();

	}
	return convert_error();
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer (effector &_master ) : common::servo_buffer(_master), master(_master)
{}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface (void)
{
	// tablica pradow maksymalnych d;a poszczegolnych osi
	int max_current [IRP6_ON_TRACK_NUM_OF_SERVOS] = {
			IRP6_ON_TRACK_AXIS_1_MAX_CURRENT, IRP6_ON_TRACK_AXIS_2_MAX_CURRENT,
			IRP6_ON_TRACK_AXIS_3_MAX_CURRENT, IRP6_ON_TRACK_AXIS_4_MAX_CURRENT,
			IRP6_ON_TRACK_AXIS_5_MAX_CURRENT, IRP6_ON_TRACK_AXIS_6_MAX_CURRENT,
			IRP6_ON_TRACK_AXIS_7_MAX_CURRENT, IRP6_ON_TRACK_AXIS_8_MAX_CURRENT
	};

	hi = new hardware_interface(master, IRQ_REAL, INT_FREC_DIVIDER, HI_RYDZ_INTR_TIMEOUT_HIGH, FIRST_SERVO_PTR,
			INTERRUPT_GENERATOR_SERVO_PTR, ISA_CARD_OFFSET, max_current);

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1
	// regulator_ptr[0] = new NL_regulator_1 (0, 0, 0.64, 16.61/5., 15.89/5., 0.35);
	regulator_ptr[0] = new NL_regulator_1_irp6ot (0, 0, 0.333, 6.2, 5.933, 0.35, master);
	// Serwomechanizm 2
	// regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
	regulator_ptr[1] = new NL_regulator_2_irp6ot (0, 0, 0.429, 6.834, 6.606, 0.35, master);
	// Serwomechanizm 3
	regulator_ptr[2] = new NL_regulator_3_irp6ot (0, 0, 0.64, 9.96/4, 9.54/4, 0.35, master);
	// Serwomechanizm 4
	// regulator_ptr[3] = new NL_regulator_4 (0, 0, 0.62, 9.85/4, 9.39/4, 0.35);
	regulator_ptr[3] = new NL_regulator_4_irp6ot (0, 0, 0.333, 5.693, 5.427, 0.35, master);
	// Serwomechanizm 5
	regulator_ptr[4] = new NL_regulator_5_irp6ot (0, 0, 0.56, 7.98/2, 7.55/2, 0.35, master);
	// Serwomechanizm 6
	// regulator_ptr[5] = new NL_regulator_6 (0, 0, 0.3079*2, 0.6, 0.6, 0.35);
	regulator_ptr[5] = new NL_regulator_6_irp6ot (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);

	regulator_ptr[6] = new NL_regulator_7_irp6ot (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);

	regulator_ptr[7] = new NL_regulator_8_irp6ot (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);

	send_after_last_step = false;
	clear_reply_status();
	clear_reply_status_tmp();
}



/*-----------------------------------------------------------------------*/
void servo_buffer::synchronise (void)
{
	const int NS = 10;     // liczba krokow rozpedzania/hamowania
	common::regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

	double synchro_step = 0.0;   // zadany przyrost polozenia

	if(master.test_mode)
	{
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();
		reply_to_EDP_MASTER();
		return;
	}

	for (int j = 0; j < master.number_of_servos; j++)
	{
		command.parameters.move.abs_position[j]=0.0;
	} // end: for


	// szeregowa synchronizacja serwomechanizmow
	for (int k = 0; k < master.number_of_servos; k++)
	{
		int j = ((k+IRP6OT_SYN_INIT_AXE)%master.number_of_servos);

		// printf("os synchronizopwana: %d \n",j);
		for (int l= 0; l < master.number_of_servos; l++)
		{
			int i = ((l+IRP6OT_SYN_INIT_AXE)%master.number_of_servos);
			// zerowy przyrost polozenia dla wszystkich napedow procz j-tego
			if ( i == j)
			{
				crp = regulator_ptr[i];
				// W.S.        crp->insert_new_step(lib::SYNCHRO_STEP_COARSE);
				switch (i)
				{
				case IRP6OT_GRIPPER_CATCH_AXE:
					synchro_step = IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_COARSE/NS;
					break;
				case IRP6OT_GRIPPER_TURN_AXE:
					synchro_step = IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_COARSE/NS;
					break;
				default:
					synchro_step = IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_COARSE/NS;
					break;
				}

				crp->insert_new_step(synchro_step);
			}
			else
			{
				regulator_ptr[i]->insert_new_step(0.0);
			}
		}

		clear_reply_status();
		clear_reply_status_tmp();

		synchro_step=0.0;

		// ruch do wykrycia wylacznika synchronizacji
		for (;;)
		{
			do
			{
				switch (j)
				{
				case IRP6OT_GRIPPER_CATCH_AXE:
					if (synchro_step > IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_COARSE )
					{
						synchro_step += IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_COARSE/NS;
						crp->insert_new_step(synchro_step);
					}
					break;
				case IRP6OT_GRIPPER_TURN_AXE:
					if (synchro_step > IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_COARSE )
					{
						synchro_step += IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_COARSE/NS;
						crp->insert_new_step(synchro_step);
					}
					break;
				default:
					if (synchro_step > IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_COARSE )
					{
						synchro_step += IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_COARSE/NS;
						crp->insert_new_step(synchro_step);
					}
					break;
				}

				//		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
			}
			while ( Move_1_step() == NO_ERROR_DETECTED ); // end: while
				//		printf("aaa: %d, %x\n", j, reply_status_tmp.error0);
			// analiza przyslanego bledu (czy wjechano na wylacznik synchronizacji?)
			// jezeli nie, to blad
			switch ( (reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL )
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
		synchro_step=0.0;
		crp->insert_new_step(synchro_step);
		for (int i = 0; i < 250; i++)
		{
			Move_1_step();
			//  printf("aabb: %d, %x\n", j, reply_status_tmp.error0);
			switch ( (reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL )
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
		switch (j)
		{
		case IRP6OT_GRIPPER_CATCH_AXE:
			synchro_step = -IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_FINE/NS;
			break;
		case IRP6OT_GRIPPER_TURN_AXE:
			synchro_step = -IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_FINE/NS;
			break;
		default:
			synchro_step = -IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_FINE/NS;
			break;
		}


		crp->insert_new_step(synchro_step);

		// wlaczenie sledzenia zera rezolwera (synchronizacja osi)
		hi->start_synchro (j);
		delay(1);
		Move_1_step();
		while (1)
		{
			//  		printf("babb: %d\n", j);
			Move_1_step();
			// W.S. -----------------------------------------------------
			//	printf("ccc: %d\n", j);
			switch (j)
			{
			case IRP6OT_GRIPPER_CATCH_AXE:
				if (synchro_step < -IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_FINE )
				{
					synchro_step -= IRP6_ON_TRACK_AXIS_7_SYNCHRO_STEP_FINE / NS;
					crp->insert_new_step(synchro_step);
				}
				break;
			case IRP6OT_GRIPPER_TURN_AXE:
				if (synchro_step < -IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_FINE )
				{
					synchro_step -= IRP6_ON_TRACK_AXIS_6_SYNCHRO_STEP_FINE / NS;
					crp->insert_new_step(synchro_step);
				}
				break;
			default:
				if (synchro_step < -IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_FINE )
				{
					synchro_step -= IRP6_ON_TRACK_AXIS_0_TO_5_SYNCHRO_STEP_FINE / NS;
					crp->insert_new_step(synchro_step);
				}
				break;
			}


			// W.S. -----------------------------------------------------
			//    	   printf("bbbb if: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
			switch ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL)
			{
			case lib::SYNCHRO_SWITCH_ON:
				//    	printf("bcbb:�SYNCHRO_SWITCH_ON\n");
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
		switch ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) )
		{
		case lib::SYNCHRO_ZERO: // zjechano z wylacznika synchronizacji i SYNCHRO_ZERO jest od razu
			//     printf("SYNCHRO_ZERO\n");
			hi->finish_synchro (j);

			//	 printf("SYNCHRO_ZERO\n");
			hi->reset_position(j);
			crp->clear_regulator();
			delay(1);
			continue;
		case OK:
			// ruch do wykrycia zera rezolwera
			//    printf("OK\n");
			for (;;)
			{
				Move_1_step();
				//        printf("OK Move_1_step\n");
				//     if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL) != OK)
				// by Y - wyciecie SYNCHRO_SWITCH_ON - ze wzgledu na wystepujace drgania
				if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739DULL) != OK)
				{
					//	   printf("OK os: %d, if: %llx, %llx\n", j, reply_status_tmp.error0, ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL));
					// Usuniecie bitow SYNCHRO_ZERO i SYNCHRO_SWITCH_ON z wszystkich osi
					// oprocz synchronizowanej
					// osie zsynchronizowane nie sa analizowane
					break;
				}
			}
			//      if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) != lib::SYNCHRO_ZERO) {
			// by Y - wyciecie SYNCHRO_SWITCH_ON
			if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001DULL) != lib::SYNCHRO_ZERO)
			{
				//	  printf("OK convert_error\n");
				convert_error();
				reply_status.error0 = reply_status_tmp.error0 | SYNCHRO_ERROR;
				reply_status.error1 = reply_status_tmp.error1;
				clear_reply_status_tmp();
				// Wypelnic servo_data
				reply_to_EDP_MASTER();
				return;
			}
			else
			{
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
	for (int k = 0; k < master.number_of_servos; k++)
	{
		int j = ((k+IRP6OT_SYN_INIT_AXE)%master.number_of_servos);
		synchro_step=0.0;
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
servo_buffer::~servo_buffer(void)
{
	for(int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++) {
		delete regulator_ptr[i];
	}

	delete hi;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions (void)
{
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	for (int i = 0; i < IRP6_ON_TRACK_NUM_OF_SERVOS; i++)
	{

		switch (i)
		{
		case IRP6OT_GRIPPER_CATCH_AXE:
			servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION;
			break;
		case IRP6OT_GRIPPER_TURN_AXE:
			servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION;
			break;
		default:
			servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION;
			break;
		}

		// przyrost polozenia w impulsach
		servo_data.position[i]  = regulator_ptr[i]->get_position_inc(1);
		servo_data.current[i]   = regulator_ptr[i]->get_meassured_current();
		servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
		servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
		servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
	}

	// przepisanie stanu regulatora chwytaka do bufora odpowiedzi dla EDP_master
	servo_data.gripper_reg_state = regulator_ptr[7]->get_reg_state();

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint64_t servo_buffer::compute_all_set_values (void)
{
	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
	uint64_t status = OK; // kumuluje numer bledu

	for (int j = 0; j < IRP6_ON_TRACK_NUM_OF_SERVOS; j++)
	{
		if (master.test_mode)
		{
			switch (j)
			{
			case IRP6OT_GRIPPER_CATCH_AXE:
				regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
						*IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION/(2*M_PI));
				break;
			case IRP6OT_GRIPPER_TURN_AXE:
				regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
						*IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION/(2*M_PI));
				break;
			default:
				regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
						*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI));
				break;
			}

		}
		else
		{
			regulator_ptr[j]->insert_new_pos_increment(hi->get_increment(j));
			regulator_ptr[j]->insert_meassured_current(hi->get_current(j));
		}
		// obliczenie nowej wartosci zadanej dla napedu
		status |= ((uint64_t) regulator_ptr[j]->compute_set_value()) << 2*j;
		// przepisanie obliczonej wartosci zadanej do hardware interface
		hi->insert_set_value(j, regulator_ptr[j]->get_set_value());
	}
	return status;
}
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
NL_regulator_1_irp6ot::NL_regulator_1_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_2_irp6ot::NL_regulator_2_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_3_irp6ot::NL_regulator_3_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_4_irp6ot::NL_regulator_4_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_5_irp6ot::NL_regulator_5_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
	first = true;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_6_irp6ot::NL_regulator_6_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_7_irp6ot::NL_regulator_7_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_8_irp6ot::NL_regulator_8_irp6ot (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{

	reg_state = next_reg_state = prev_reg_state = lib::GRIPPER_START_STATE;
	sum_of_currents = current_index = 0;
	for (int i=0; i < IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS; i++)
	{
		currents [i] = 0;
	}

	display=0;
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_1_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	//   struct timespec step_time;

	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_1_MAX_ACC, common::SG_REG_1_MAX_SPEED);

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	/*
    if (!aaa)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new ) > 1) ) {
      aaa++;
     }
	 */
	// if (aaa > 0 && aaa < 30 ) {
	//  cprintf("O1: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O1: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[0].adr_offset_plus_4,md.robot_status[0].adr_offset_plus_6);
	//  aaa++;
	//  if (aaa == 9) aaa=0;
	// }

	// by Y - bez sensu
	// Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
	if (fabs(position_increment_new) > MAX_INC)
		position_increment_new = position_increment_old;


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005*(step_new_pulse- position_increment_new) -
	0.995*(step_old_pulse - position_increment_old);

	// if (fabs(step_new_pulse) > 70.0) {
	//  cprintf("snp = %lf   pin = %lf\n",step_new_pulse, position_increment_new);
	// }

	// if (fabs(delta_eint) > 50.0) {
	//  cprintf("%4.0lf ",delta_eint);
	// }

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.4152;
				b0 = 0.9017*1.5;
				b1 = 0.7701*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.4152;
				b0 = 0.9017*1.0;
				b1 = 0.7701*1.0;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		if ( ( fabs(set_value_new)) < 0.1 )
			counter++;
		else
			counter = 0;

		if ( fabs(step_new) < EPS && fabs(position_increment_new) < EPS && (counter > integrator_off) )
		{
			set_value_new = (1+a)*set_value_old - a*set_value_very_old +
			b0*(step_new_pulse - position_increment_new)
			- b1*(step_old_pulse - position_increment_old);
		}
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}

	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[0] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[0] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[0] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[0]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE1_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE1_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE1_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE1_MAX_PWM_INCREMENT;



	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value =(int) set_value_new;

	return alg_par_status;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_2_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	//
	// step_old_pulse;
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)


	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow


	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_2_MAX_ACC, common::SG_REG_2_MAX_SPEED, false);


	// przeliczenie radianow na impulsy
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);

	/*
    if (!bbb)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      bbb++;
     }
	 */
	// if (bbb > 0 && bbb < 10 ) {
	//  cprintf("O2: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O2: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[1].adr_offset_plus_4,md.robot_status[1].adr_offset_plus_6);
	//  bbb++;
	//  if (bbb == 9) bbb=0;
	// }

	// by Y - bez sensu
	// Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
	if (fabs(position_increment_new) > MAX_INC)
		position_increment_new = position_increment_old;


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005*(step_new_pulse - position_increment_new) -
	0.995*(step_old_pulse - position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 2.3100*1.5;
				b1 = 2.0312*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 2.3100*2.0;
				b1 = 2.0312*2.0;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}

	// printf("banana1: %f\n", set_value_new);

	master.rb_obj->lock_mutex();


	master.rb_obj->step_data.desired_inc[1] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[1] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[1] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[1]=(float) (step_new_pulse - position_increment_new);


	master.rb_obj->unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE2_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE2_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE2_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE2_MAX_PWM_INCREMENT;

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_3_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow


	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_3_MAX_ACC, common::SG_REG_3_MAX_SPEED);

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	/*
    if (!ccc)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      ccc++;
     }
	 */
	// if (ccc > 0 && ccc < 10 ) {
	//  cprintf("O3: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O3: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[2].adr_offset_plus_4,md.robot_status[2].adr_offset_plus_6);
	//  ccc++;
	//  if (ccc == 9) ccc=0;
	// }

	// by Y - bez sensu
	// Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
	if (fabs(position_increment_new) > MAX_INC)
		position_increment_new = position_increment_old;


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005*(step_new_pulse - position_increment_new) -
	0.995*(step_old_pulse - position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.4152;
				b0 = 1.2500*1.5;
				b1 = 1.0998*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.4152;
				b0 = 1.2500*2.5;
				b1 = 1.0998*2.5;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}


	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[2] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[2] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[2] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[2]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE3_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE3_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE3_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE3_MAX_PWM_INCREMENT;

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_4_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow


	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_4_MAX_ACC, common::SG_REG_4_MAX_SPEED);

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);

	/*
    if (!ddd)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      ddd++;
     }
	 */
	// if (ddd > 0 && ddd < 10 ) {
	//  cprintf("O4: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O4: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[3].adr_offset_plus_4,md.robot_status[3].adr_offset_plus_6);
	//  ddd++;
	//  if (ddd == 9) ddd=0;
	// }

	// by Y - bez sensu
	// Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
	if (fabs(position_increment_new) > MAX_INC)
		position_increment_new = position_increment_old;


	servo_pos_increment_new_sum += position_increment_new;// by Y
	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005*(step_new_pulse - position_increment_new) -
	0.995*(step_old_pulse - position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*1.0;
				b1 = 0.9166*1.0;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*2.5;
				b1 = 0.9166*2.5;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				; // end: switch (algorithm_parameters_no)
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}

	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[3] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[3] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[3] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);
	// master.rb_obj->step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE4_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE4_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE4_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE4_MAX_PWM_INCREMENT;

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	return alg_par_status;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_5_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_5_MAX_ACC, common::SG_REG_5_MAX_SPEED);

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);

	/*
    if (!eee)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      eee++;
     }
	 */
	// if (eee > 0 && eee < 10 ) {
	//  cprintf("O5: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O5: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[4].adr_offset_plus_4,md.robot_status[4].adr_offset_plus_6);
	//  eee++;
	//  if (eee == 9) eee=0;
	// }
	// by Y - bez sensu
	// Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
	if (fabs(position_increment_new) > MAX_INC)
		position_increment_new = position_increment_old;


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005*(step_new_pulse - position_increment_new) -
	0.995*(step_old_pulse - position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*1.5;
				b1 = 0.9166*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*2.5;
				b1 = 0.9166*2.5;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a =  0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}

	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[4] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[4] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[4] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[4]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE5_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE5_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE5_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE5_MAX_PWM_INCREMENT;

	// if (fabs(set_value_new) > 200.0 && first) {
	// cprintf("PIN=%lf PIO=%lf DIN=%lf DIO=%lf SO=%lf SVVO=%lf SV0=%lf\n", position_increment_new,
	//     position_increment_old, delta_eint, delta_eint_old,
	//     step_old_pulse, set_value_very_old, set_value_old);
	// first = false;
	// }

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	return alg_par_status;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_6_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_6_MAX_ACC, common::SG_REG_6_MAX_SPEED);

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);

	/*
    if (!fff)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      fff++;
     }
	 */
	// if (fff > 0 && fff < 10 ) {
	//  cprintf("O6: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O6: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[5].adr_offset_plus_4,md.robot_status[5].adr_offset_plus_6);
	//  fff++;
	//  if (fff == 9) fff=0;
	// }


	// by Y - bez sensu
	// Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
	if (fabs(position_increment_new) > MAX_INC)
		position_increment_new = position_increment_old;


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005*(step_new_pulse - position_increment_new) -
	0.995*(step_old_pulse - position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a = 0.3079;
				b0 = 1.0942*1.5;
				b1 = 0.9166*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*2.5;
				b1 = 0.9166*2.5;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}
	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE6_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE6_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE6_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE6_MAX_PWM_INCREMENT;

	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[5] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[5] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[5] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[5]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// if (set_value_new > 0.0) {
	//  cprintf("svn = %lf  pin = %lf\n",set_value_new, position_increment_new);
	// }

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	return alg_par_status;

}
/*-----------------------------------------------------------------------*/



/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_7_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenia
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenia
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// double root_position_increment_new=position_increment_new;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_7_MAX_ACC, common::SG_REG_7_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*IRP6_POSTUMENT_AXIS_6_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_6_INC_PER_REVOLUTION/(2*M_PI);
	//position_increment_new= position_increment_new/AXE_6_POSTUMENT_TO_TRACK_RATIO;

	// if (step_new!=0.0) printf(" 7 reg:%f\n", step_new);

	/*
    if (!fff)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      fff++;
     }
	 */
	// if (fff > 0 && fff < 10 ) {
	//  cprintf("O6: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O6: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[5].adr_224,md.robot_status[5].adr_226);
	//  fff++;
	//  if (fff == 9) fff=0;
	// }


	// Jesli przyrost jest wiekszy od dopuszczalnego
	/* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
	 */


	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	//pos_increment_new_sum += root_position_increment_new;
	//servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.020*(step_new_pulse - position_increment_new) -
	0.980*(step_old_pulse - position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0
			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a = 0.3079;
				b0 = 1.0942*1.5;
				b1 = 0.9166*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*2.5;
				b1 = 0.9166*2.5;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1
				switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}

	/*
    a=0.391982182628;
    b0=6.537527839644;
    b1=5.990311804009;
	 */

	a=0.3;
	b0=1.364*POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //4
	b1=1.264*POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //1.364//4

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = ((1+a)*set_value_old - a*set_value_very_old +
				b0*delta_eint - b1*delta_eint_old);
		break;
	case 1:  // algorytm nr 1
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*(step_new_pulse - position_increment_new)
		- b1*(step_old_pulse - position_increment_old);
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;


	// if (set_value_new!=0.0) printf ("aa: %f\n", set_value_new);

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE7_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE7_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE7_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE7_MAX_PWM_INCREMENT;

	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[6] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[6] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[6] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[6]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// if (set_value_new > 0.0) {
	//  cprintf("svn = %lf  pin = %lf\n",set_value_new, position_increment_new);
	// }

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	return alg_par_status;

}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_8_irp6ot::compute_set_value (void)
{
	// algorytm regulacji dla serwomechanizmu

	// position_increment_old - przedostatnio odczytany przyrost polozenie
	//                         (delta y[k-2] -- mierzone w impulsach)
	// position_increment_new - ostatnio odczytany przyrost polozenie
	//                         (delta y[k-1] -- mierzone w impulsach)
	// step_old_pulse               - poprzednia wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-2] -- mierzone w impulsach)
	// step_new               - nastepna wartosc zadana dla jednego kroku
	//                         regulacji (przyrost wartosci zadanej polozenia --
	//                         delta r[k-1] -- mierzone w radianach)
	// set_value_new          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k]): czas trwania jedynki
	// set_value_old          - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-1]): czas trwania jedynki
	// set_value_very_old     - wielkosc kroku do realizacji przez HIP
	//                         (wypelnienie PWM -- u[k-2]): czas trwania jedynki

	double step_new_pulse; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia --
	// delta r[k-1] -- mierzone w impulsach)
	uint8_t alg_par_status;   // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	alg_par_status = ALGORITHM_AND_PARAMETERS_OK;

	// double root_position_increment_new=position_increment_new;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_8_MAX_ACC, common::SG_REG_8_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*IRP6_ON_TRACK_AXIS_7_INC_PER_REVOLUTION/(2*M_PI);//*AXE_7_POSTUMENT_TO_TRACK_RATIO);
	//position_increment_new= position_increment_new/AXE_7_POSTUMENT_TO_TRACK_RATIO;

	// printf("bbb: %d\n",  meassured_current);


	// if (step_new!=0.0) printf(" 8 reg:%f\n", step_new);

	/*
    if (!fff)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      fff++;
     }
	 */
	// if (fff > 0 && fff < 10 ) {
	//  cprintf("O6: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O6: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[5].adr_224,md.robot_status[5].adr_226);
	//  fff++;
	//  if (fff == 9) fff=0;
	// }


	/* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
	 */


	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	//pos_increment_new_sum += root_position_increment_new;
	// servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.020*(step_new_pulse - position_increment_new) -
	0.980*(step_old_pulse - position_increment_old);




	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ( (current_algorithm_no != algorithm_no) ||
			(current_algorithm_parameters_no != algorithm_parameters_no) )
	{
		switch (algorithm_no)
		{
		case 0:  // algorytm nr 0


			switch (algorithm_parameters_no)
			{
			case 0:  // zestaw parametrow nr 0
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a = 0.3079;
				b0 = 1.0942*1.5;
				b1 = 0.9166*1.5;
				k_feedforward = 0.35;
				break;
			case 1: // zestaw parametrow nr 1
				current_algorithm_parameters_no = algorithm_parameters_no;
				current_algorithm_no = algorithm_no;
				a  = 0.3079;
				b0 = 1.0942*2.5;
				b1 = 0.9166*2.5;
				k_feedforward = 0;
				break;
			default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
				break;
			}
			break;
			case 1: // algorytm nr 1

						switch (algorithm_parameters_no)
				{
				case 0: // zestaw parametrow nr 0
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				case 1: // zestaw parametrow nr 1
					current_algorithm_parameters_no = algorithm_parameters_no;
					current_algorithm_no = algorithm_no;
					a  = 0;
					b0 = 0;
					b1 = 0;
					k_feedforward = 0;
					break;
				default: // blad - nie ma takiego zestawu parametrow dla tego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
					break;
				}
				; // end: switch (algorithm_parameters_no)
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}
	/*
    a=0.391982182628;
    b0=6.537527839644;
    b1=5.990311804009;
	 */

	a=0.2; //0.3
	b0=15.984375*POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //15.984375; //3
	b1=15.784375*POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //15.984375; //3




	//14.4
	//a=0.2;
	//b0=15.984375;
	//b1=15.984375;

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0


	//	if (meassured_current != 0) fprintf(stdout,"alg 0: %d\n", meassured_current);
/*
	    display++;
		        if (display >= 500)
		        {
		            display = 0;
		            fprintf(stdout,"alg 0: %d\n", meassured_current);
		        }
*/

		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = ((1+a)*set_value_old - a*set_value_very_old +
				b0*delta_eint - b1*delta_eint_old);
		break;
	case 1:  // algorytm nr 1
		if (meassured_current != 0) fprintf(stdout,"alg 1: %d\n", meassured_current);
		/*
	    display++;
		        if (display >= 500)
		        {
		            display = 0;
		            fprintf(stdout,"alg 1: %d\n", meassured_current);
		        }
*/

		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = 50;
		break;
	default: // w tym miejscu nie powinien wystapic blad zwiazany z
		// nieistniejacym numerem algorytmu
		set_value_new = 0; // zerowe nowe sterowanie
		break;
	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	/*
    #define MAXX_PWM 250
      // ograniczenie na sterowanie
      if (set_value_new > MAXX_PWM)
         set_value_new = MAXX_PWM;
      if (set_value_new < -MAXX_PWM)
         set_value_new = -MAXX_PWM;

	 */

	//   if (set_value_new!=0.0) printf ("aa: %f\n", set_value_new);

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > IRP6_ON_TRACK_AXE8_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + IRP6_ON_TRACK_AXE8_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -IRP6_ON_TRACK_AXE8_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - IRP6_ON_TRACK_AXE8_MAX_PWM_INCREMENT;

	master.rb_obj->lock_mutex();

	master.rb_obj->step_data.desired_inc[7] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj->step_data.current_inc[7] = (short int) position_increment_new;
	master.rb_obj->step_data.pwm[7] = (float) set_value_new;
	master.rb_obj->step_data.uchyb[7]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj->unlock_mutex();

	// if (set_value_new > 0.0) {
	//  cprintf("svn = %lf  pin = %lf\n",set_value_new, position_increment_new);
	// }

	// przepisanie nowych wartosci zmiennych do zmiennych przechowujacych wartosci poprzednie
	position_increment_old = position_increment_new;
	delta_eint_old = delta_eint;
	step_old_pulse = step_new_pulse;
	set_value_very_old = set_value_old;
	set_value_old = set_value_new;
	PWM_value = (int) set_value_new;

	//	printf("CC: PWM: %d, %d, %d, %d\n", PWM_value, meassured_current, reg_state, kk);

	// AUTOMAT ZABEZPIECZAJACY SILNIK CHWYTAKA PRZED PRZEGRZANIEM

	// wyznaczenie pradu na zalozonych horyzoncie wstecz
	if (master.step_counter > IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS)
	{
		sum_of_currents -= currents [current_index];
	}

	sum_of_currents += meassured_current;

	currents [current_index] = meassured_current;

	current_index = ((++current_index)%IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS);


	reg_state=next_reg_state;

	//	printf("aa: %d, %d, %d\n",  sum_of_currents, meassured_current, kk);
	//printf("aa: %d\n", sum_of_currents);

	switch (reg_state)
	{
	case lib::GRIPPER_START_STATE:

		if (sum_of_currents > IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE)
		{
			next_reg_state = lib::GRIPPER_BLOCKED_STATE;
			gripper_blocked_start_time = master.step_counter;
			//		printf("gripper GRIPPER_BLOCKED_STATE state\n");
		}
		break;

	case lib::GRIPPER_BLOCKED_STATE:
	//	fprintf(stdout,"GRIPPER_BLOCKED_STATE %d\n", meassured_current);
		if (((master.step_counter - gripper_blocked_start_time) > GRIPPER_BLOCKED_TIME_PERIOD)
				&& (!(sum_of_currents > IRP6_ON_TRACK_GRIPPER_SUM_OF_CURRENTS_MAX_VALUE)))
		{
			//			printf("gripper GRIPPER_START_STATE state\n");
			next_reg_state = lib::GRIPPER_START_STATE;
		}
		else
		{
			position_increment_old = 0;
			position_increment_new =  0;
			delta_eint_old =  0;
			delta_eint =  0;
			step_old_pulse = 0;
			step_new_pulse = 0;
			set_value_very_old =  0;
			set_value_old = 0;
			set_value_old =  0;
			set_value_new =  0;
		}
		break;

	default:
		break;
	}

	prev_reg_state=reg_state;

	return alg_par_status;
}
/*-----------------------------------------------------------------------*/

} // namespace irp6ot


namespace common {

servo_buffer* return_created_servo_buffer (manip_and_conv_effector &_master)
{
	return new irp6ot::servo_buffer ((irp6ot::effector &)(_master));
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
