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

#include "edp/polycrank/edp_e_polycrank.h"
#include "edp/polycrank/hi_polycrank.h"
#include "edp/polycrank/sg_polycrank.h"

// extern edp_irp6m_effector* master;   // Bufor polecen i odpowiedzi EDP_MASTER

// uint64_t kk;	// numer pomiaru od momentu startu pomiarow

namespace mrrocpp {
namespace edp {
namespace polycrank {

/*-----------------------------------------------------------------------*/
uint8_t servo_buffer::Move_a_step (void)
{
	// wykonac ruch o krok nie reagujac na SYNCHRO_SWITCH ora SYNCHRO_ZERO

	Move_1_step ();
	if (master.is_synchronised())
	{// by Y aktualizacja transformera am jedynie sens po synchronizacji (kiedy robot zna swoja pozycje)
		// by Y - do dokonczenia
		for (int i=0; i < POLYCRANK_NUM_OF_SERVOS; i++)
		{
			if (!(master.test_mode))
			{
				master.update_servo_current_motor_pos_abs(hi->get_position(i)*(2*M_PI)/POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION, i);
			}
		}

		master.servo_joints_and_frame_actualization_and_upload();// by Y - aktualizacja trasformatora
	}
	return convert_error();
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer (effector &_master) : common::servo_buffer(_master), master(_master)
{

	hi = new hardware_interface(_master);

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1

	// regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
	// kolumna dla irp6 polycrank
	regulator_ptr[0] = new NL_regulator_2_irp6m (0, 0, 0.429, 6.834, 6.606, 0.35, master); // kolumna dla irp6 polycrank

	regulator_ptr[1] = new NL_regulator_3_irp6m (0, 0, 0.64, 9.96/4, 9.54/4, 0.35, master);

	// regulator_ptr[3] = new NL_regulator_4 (0, 0, 0.62, 9.85/4, 9.39/4, 0.35);
	regulator_ptr[2] = new NL_regulator_4_irp6m (0, 0, 0.333, 5.693, 5.427, 0.35, master);

	regulator_ptr[3] = new NL_regulator_5_irp6m (0, 0, 0.56, 7.98/2, 7.55/2, 0.35, master);

	// regulator_ptr[5] = new NL_regulator_6 (0, 0, 0.3079*2, 0.6, 0.6, 0.35);
	regulator_ptr[4] = new NL_regulator_6_irp6m (0, 0, 0.39, 8.62/2., 7.89/2., 0.35, master);
	// regulator_ptr[0] = new NL_regulator_1 (0, 0, 0.64, 16.61/5., 15.89/5., 0.35);


	send_after_last_step = false;
	clear_reply_status();
	clear_reply_status_tmp();

	for (int j = 0; j < POLYCRANK_NUM_OF_SERVOS; j++)
	{
		command.parameters.move.abs_position[j]=0.0;
	}
}
/*-----------------------------------------------------------------------*/




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

	for (int j = 0; j < (POLYCRANK_NUM_OF_SERVOS); j++)
	{
		command.parameters.move.abs_position[j]=0.0;
	}


	// szeregowa synchronizacja serwomechanizmow
	for (int k = 0; k < (POLYCRANK_NUM_OF_SERVOS); k++)
	{
		int j = ((k+IRP6M_SYN_INIT_AXIS)%(POLYCRANK_NUM_OF_SERVOS));

		// printf("os synchronizopwana: %d \n",j);
		for (int l= 0; l < (POLYCRANK_NUM_OF_SERVOS); l++)
		{
			int i = ((l+IRP6M_SYN_INIT_AXIS)%(POLYCRANK_NUM_OF_SERVOS));
			// zerowy przyrost polozenia dla wszystkich napedow procz j-tego
			if ( i == j)
			{
				crp = regulator_ptr[i];
				// W.S.        crp->insert_new_step(lib::SYNCHRO_STEP_COARSE);

				synchro_step = POLYCRANK_SYNCHRO_STEP_COARSE/NS;

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

				if (synchro_step > POLYCRANK_SYNCHRO_STEP_COARSE )
				{
					synchro_step += POLYCRANK_SYNCHRO_STEP_COARSE/NS;
					crp->insert_new_step(synchro_step);
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
		synchro_step = -POLYCRANK_SYNCHRO_STEP_FINE/NS;

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

			if (synchro_step < -POLYCRANK_SYNCHRO_STEP_FINE )
			{
				synchro_step -= POLYCRANK_SYNCHRO_STEP_FINE / NS;
				crp->insert_new_step(synchro_step);
			}


			// W.S. -----------------------------------------------------
			//    	   printf("bbbb if: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
			switch ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL)
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
				//      if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL) != OK)
				// by Y - wyciecie SYNCHRO_SWITCH_ON - ze wzgledu na wystepujace drgania
				if ( ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739DULL) != OK)
				{
					//   printf("OK os: %d, if: %llx, %llx\n", j, reply_status_tmp.error0, ((reply_status_tmp.error0 >> (5*j)) & 0xCE739CE739CE739FULL));
					// Usuniecie bitow SYNCHRO_ZERO i SYNCHRO_SWITCH_ON z wszystkich osi
					// oprocz synchronizowanej
					// osie zsynchronizowane nie sa analizowane
					break;
				}
			}
			//     if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL) != lib::SYNCHRO_ZERO) {
			// by Y - wyciecie SYNCHRO_SWITCH_ON
			if ( ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001DULL) != lib::SYNCHRO_ZERO)
			{
				// 	  printf("OK convert_error: %llx\n", ((reply_status_tmp.error0 >> (5*j)) & 0x000000000000001FULL));
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
	for (int k = 0; k < (POLYCRANK_NUM_OF_SERVOS); k++)
	{
		int j = ((k+IRP6M_SYN_INIT_AXIS)%(POLYCRANK_NUM_OF_SERVOS));
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
{}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions (void)
{
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	for (int i = 0; i < POLYCRANK_NUM_OF_SERVOS; i++)
	{

		servo_data.abs_position[i]  = hi->get_position(i)*(2*M_PI)/POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION;


		// przyrost polozenia w impulsach
		servo_data.position[i]  = regulator_ptr[i]->get_position_inc(1);
		servo_data.current[i]   = regulator_ptr[i]->get_meassured_current();
		servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
		servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
		servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
	}

}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint64_t servo_buffer::compute_all_set_values (void)
{
	// obliczenie nastepnej wartosci zadanej dla wszystkich napedow
	uint64_t status = OK; // kumuluje numer bledu


	for (int j = 0; j < POLYCRANK_NUM_OF_SERVOS; j++)
	{
		if (master.test_mode)
		{
			regulator_ptr[j]->insert_new_pos_increment(regulator_ptr[j]->return_new_step()
					*POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI));
		}
		else
		{
			regulator_ptr[j]->insert_meassured_current(hi->get_current(j));
			regulator_ptr[j]->insert_new_pos_increment(hi->get_increment(j));
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
NL_regulator_2_irp6m::NL_regulator_2_irp6m (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_3_irp6m::NL_regulator_3_irp6m (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_4_irp6m::NL_regulator_4_irp6m (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_5_irp6m::NL_regulator_5_irp6m (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
	first = true;
}
/*-----------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
NL_regulator_6_irp6m::NL_regulator_6_irp6m (uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::manip_and_conv_effector &_master)
: NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/




// kolumna i tak dalej
/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_2_irp6m::compute_set_value (void)
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
	constraint_detector(common::SG_REG_2_MAX_ACC, common::SG_REG_2_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*POLYCRANK_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	//position_increment_new= position_increment_new/AXIS_0_TO_5_MECHATRONIKA_TO_TRACK_RATIO;

	// if (step_new!=0.0) printf(" 2 reg:%f\n", step_new);

	/*
    if (!bbb)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      bbb++;
     }
	 */
	// if (bbb > 0 && bbb < 10 ) {
	//  cprintf("O2: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O2: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[1].adr_224,md.robot_status[1].adr_226);
	//  bbb++;
	//  if (bbb == 9) bbb=0;
	// }
	/* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
	 */

	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	//pos_increment_new_sum += root_position_increment_new;
	//servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	// pos_increment_new_sum += position_increment_new;
	// servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.010*(step_new_pulse - position_increment_new) -
	0.990*(step_old_pulse - position_increment_old);

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

	a=0.412429378531;
	b0=2.594932; //stara z przelicz rezolwer/enkoder 15.219541375872
	b1=2.504769; //

	switch (algorithm_no)
	{
	case 0:  // algorytm nr 0
		// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
		set_value_new = (1+a)*set_value_old - a*set_value_very_old +
		b0*delta_eint - b1*delta_eint_old;
		// cout<<a<<" "<<b0<<" "<<b1<<"\n";
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

	master.rb_obj.lock_mutex();

	master.rb_obj.step_data.desired_inc[1] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj.step_data.current_inc[1] = (short int) position_increment_new;
	master.rb_obj.step_data.pwm[1] = (float) set_value_new;
	master.rb_obj.step_data.uchyb[1]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj.unlock_mutex();

	//  	set_value_new=set_value_new;

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > POLYCRANK_AXIS2_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + POLYCRANK_AXIS2_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -POLYCRANK_AXIS2_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - POLYCRANK_AXIS2_MAX_PWM_INCREMENT;

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
uint8_t NL_regulator_3_irp6m::compute_set_value (void)
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
	constraint_detector(common::SG_REG_3_MAX_ACC, common::SG_REG_3_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*POLYCRANK_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	///position_increment_new= position_increment_new;

	// if (step_new!=0.0) printf(" 3 reg:%f\n", step_new);


	/*
    if (!ccc)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      ccc++;
     }
	 */

	// if (ccc > 0 && ccc < 10 ) {
	//  cprintf("O3: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O3: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[2].adr_224,md.robot_status[2].adr_226);
	//  ccc++;
	//  if (ccc == 9) ccc=0;
	// }

	/* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
	 */

	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	// pos_increment_new_sum += root_position_increment_new;
	// servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.008*(step_new_pulse - position_increment_new) -
	0.992*(step_old_pulse - position_increment_old);

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

	a=0.655629139073;
	b0=1.030178; //6.042100283822;
	b1=0.986142;

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


	master.rb_obj.lock_mutex();

	master.rb_obj.step_data.desired_inc[2] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj.step_data.current_inc[2] = (short int) position_increment_new;
	master.rb_obj.step_data.pwm[2] = (float) set_value_new;
	master.rb_obj.step_data.uchyb[2]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj.unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > POLYCRANK_AXIS3_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + POLYCRANK_AXIS3_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -POLYCRANK_AXIS3_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - POLYCRANK_AXIS3_MAX_PWM_INCREMENT;

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
uint8_t NL_regulator_4_irp6m::compute_set_value (void)
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
	constraint_detector(common::SG_REG_4_MAX_ACC, common::SG_REG_4_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*POLYCRANK_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	//position_increment_new= position_increment_new/AXIS_0_TO_5_MECHATRONIKA_TO_TRACK_RATIO;

	// if (step_new!=0.0) printf(" 4 reg:%f\n", step_new);

	/*
    if (!ddd)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      ddd++;
     }
	 */
	// if (ddd > 0 && ddd < 10 ) {
	//  cprintf("O4: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O4: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[3].adr_224,md.robot_status[3].adr_226);
	//  ddd++;
	//  if (ddd == 9) ddd=0;
	// }

	/* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
	 */

	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	//pos_increment_new_sum += root_position_increment_new;
	// servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.008*(step_new_pulse - position_increment_new) -
	0.992*(step_old_pulse - position_increment_old);

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
				break;
				default: // blad - nie ma takiego algorytmu
					// => przywrocic stary algorytm i j stary zestaw parametrow
					algorithm_no = current_algorithm_no;
					algorithm_parameters_no = current_algorithm_parameters_no;
					alg_par_status = UNIDENTIFIED_ALGORITHM_NO;
					break;
		}
	}


	a=0.315789473684;
	b0=1.997464;
	b1=1.904138;

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



	master.rb_obj.lock_mutex();

	master.rb_obj.step_data.desired_inc[3] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj.step_data.current_inc[3] = (short int) position_increment_new;
	master.rb_obj.step_data.pwm[3] = (float) set_value_new;
	master.rb_obj.step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);
	// master.rb_obj.step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj.unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > POLYCRANK_AXIS4_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + POLYCRANK_AXIS4_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -POLYCRANK_AXIS4_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - POLYCRANK_AXIS4_MAX_PWM_INCREMENT;

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
uint8_t NL_regulator_5_irp6m::compute_set_value (void)
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
	constraint_detector(common::SG_REG_5_MAX_ACC, common::SG_REG_5_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*POLYCRANK_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	//position_increment_new= position_increment_new/AXIS_0_TO_5_MECHATRONIKA_TO_TRACK_RATIO;
	/*
    if (!eee)
     if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new - position_increment_old ) > 40) ) {
      eee++;
     }
	 */
	// if (eee > 0 && eee < 10 ) {
	//  cprintf("O5: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O5: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[4].adr_224,md.robot_status[4].adr_226);
	//  eee++;
	//  if (eee == 9) eee=0;
	// }


	/* // by Y - bez sensu
    // Jesli rzeczywisty przyrost jest wiekszy od dopuszczalnego
     if (fabs(position_increment_new) > MAX_INC)
       position_increment_new = position_increment_old;
	 */

	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	// pos_increment_new_sum += root_position_increment_new;
	// servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.010*(step_new_pulse - position_increment_new) -
	0.990*(step_old_pulse - position_increment_old);

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


	a=0.548946716233;
	b0=1.576266; //9.244959545156;
	b1=1.468599; //8.613484947882;


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

	master.rb_obj.lock_mutex();

	master.rb_obj.step_data.desired_inc[4] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj.step_data.current_inc[4] = (short int) position_increment_new;
	master.rb_obj.step_data.pwm[4] = (float) set_value_new;
	master.rb_obj.step_data.uchyb[4]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj.unlock_mutex();

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

	// ograniczenie przyrostu PWM
	// ma na celu zapobiegac osiaganiu zbyt duzych pradow we wzmacniaczach mocy
	if (set_value_new - set_value_old > POLYCRANK_AXIS5_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + POLYCRANK_AXIS5_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -POLYCRANK_AXIS5_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - POLYCRANK_AXIS5_MAX_PWM_INCREMENT;

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
uint8_t NL_regulator_6_irp6m::compute_set_value (void)
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

	double root_position_increment_new=position_increment_new;

	// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie
	constraint_detector(common::SG_REG_6_MAX_ACC, common::SG_REG_6_MAX_SPEED);

	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*POLYCRANK_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new*POLYCRANK_AXIS_0_TO_5_INC_PER_REVOLUTION/(2*M_PI);
	//position_increment_new= position_increment_new/AXIS_0_TO_5_MECHATRONIKA_TO_TRACK_RATIO;

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
	// pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*MECHATRONIKA_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += root_position_increment_new;
	servo_pos_increment_new_sum += root_position_increment_new;// by Y

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

	a=0.391982182628;
	b0=1.114648; //6.537527839644;
	b1=1.021348; //5.990311804009;


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
	if (set_value_new - set_value_old > POLYCRANK_AXIS6_MAX_PWM_INCREMENT)
		set_value_new = set_value_old + POLYCRANK_AXIS6_MAX_PWM_INCREMENT;
	if (set_value_new- set_value_old < -POLYCRANK_AXIS6_MAX_PWM_INCREMENT)
		set_value_new = set_value_old - POLYCRANK_AXIS6_MAX_PWM_INCREMENT;

	master.rb_obj.lock_mutex();

	master.rb_obj.step_data.desired_inc[5] = (float) step_new_pulse; // pozycja osi 0
	master.rb_obj.step_data.current_inc[5] = (short int) position_increment_new;
	master.rb_obj.step_data.pwm[5] = (float) set_value_new;
	master.rb_obj.step_data.uchyb[5]=(float) (step_new_pulse - position_increment_new);

	master.rb_obj.unlock_mutex();

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

} // namespace polycrank

namespace common {

servo_buffer* return_created_servo_buffer (manip_and_conv_effector &_master)
{
	return new polycrank::servo_buffer ((polycrank::effector &)(_master));
}


} // namespace common
} // namespace edp
} // namespace mrrocpp





