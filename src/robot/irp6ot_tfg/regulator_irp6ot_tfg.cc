/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"

#include "robot/irp6ot_tfg/edp_irp6ot_tfg_effector.h"

// Klasa edp_irp6ot_effector.
//#include "base/edp/irp6_on_track/edp_irp6ot_effector.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
//#include "base/edp/irp6_on_track/hi_irp6ot.h"
// Klasa servo_buffer.
#include "robot/irp6ot_tfg/regulator_irp6ot_tfg.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_tfg {

/*-----------------------------------------------------------------------*/
NL_regulator_8_irp6ot::NL_regulator_8_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{

	reg_state = next_reg_state = prev_reg_state = lib::GRIPPER_START_STATE;
	sum_of_currents = current_index = 0;
	for (int i = 0; i < GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS; i++) {
		currents[i] = 0;
	}

	display = 0;
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_8_irp6ot::compute_set_value(void)
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
	uint8_t alg_par_status; // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	alg_par_status = common::ALGORITHM_AND_PARAMETERS_OK;

	// double root_position_increment_new=position_increment_new;


	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*IRP6_POSTUMENT_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new * AXIS_7_INC_PER_REVOLUTION / (2 * M_PI);//*AXE_7_POSTUMENT_TO_TRACK_RATIO);
	//position_increment_new= position_increment_new/AXE_7_POSTUMENT_TO_TRACK_RATIO;

	// printf("bbb: %d\n",  measured_current);


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
	 if (fabs(position_increment_new) > common::MAX_INC)
	 position_increment_new = position_increment_old;
	 */

	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	//pos_increment_new_sum += root_position_increment_new;
	// servo_pos_increment_new_sum += root_position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.020 * (step_new_pulse - position_increment_new) - 0.980 * (step_old_pulse
			- position_increment_old);

	// Sprawdzenie czy numer algorytmu lub zestawu parametrow sie zmienil?
	// Jezeli tak, to nalezy dokonac uaktualnienia numerow (ewentualnie wykryc niewlasciwosc numerow)
	if ((current_algorithm_no != algorithm_no) || (current_algorithm_parameters_no != algorithm_parameters_no)) {
		switch (algorithm_no)
		{
			case 0: // algorytm nr 0


				switch (algorithm_parameters_no)
				{
					case 0: // zestaw parametrow nr 0
						current_algorithm_parameters_no = algorithm_parameters_no;
						current_algorithm_no = algorithm_no;
						a = 0.3079;
						b0 = 1.0942 * 1.5;
						b1 = 0.9166 * 1.5;
						k_feedforward = 0.35;
						break;
					case 1: // zestaw parametrow nr 1
						current_algorithm_parameters_no = algorithm_parameters_no;
						current_algorithm_no = algorithm_no;
						a = 0.3079;
						b0 = 1.0942 * 2.5;
						b1 = 0.9166 * 2.5;
						k_feedforward = 0;
						break;
					default: // blad => przywrocic stary algorytm i j stary zestaw parametrow
						algorithm_no = current_algorithm_no;
						algorithm_parameters_no = current_algorithm_parameters_no;
						alg_par_status = common::UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
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
						alg_par_status = common::UNIDENTIFIED_ALGORITHM_PARAMETERS_NO;
						break;
				}
				; // end: switch (algorithm_parameters_no)
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}
	}
	/*
	 a=0.391982182628;
	 b0=6.537527839644;
	 b1=5.990311804009;
	 */

	a = 0.2; //0.3
	b0 = 15.984375 * POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //15.984375; //3
	b1 = 15.784375 * POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //15.984375; //3


	//14.4
	//a=0.2;
	//b0=15.984375;
	//b1=15.984375;

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0


			//	if (measured_current != 0) fprintf(stdout,"alg 0: %d\n", measured_current);
			/*
			 display++;
			 if (display >= 500)
			 {
			 display = 0;
			 fprintf(stdout,"alg 0: %d\n", measured_current);
			 }
			 */

			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = ((1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old);
			break;
		case 1: // algorytm nr 1
			if (measured_current != 0)
				fprintf(stdout, "alg 1: %d\n", measured_current);
			/*
			 display++;
			 if (display >= 500)
			 {
			 display = 0;
			 fprintf(stdout,"alg 1: %d\n", measured_current);
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


	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[0] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[0] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[0] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[0] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[0] = measured_current;
	}

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

	//	printf("CC: PWM: %d, %d, %d, %d\n", PWM_value, measured_current, reg_state, kk);

	// AUTOMAT ZABEZPIECZAJACY SILNIK CHWYTAKA PRZED PRZEGRZANIEM

	// wyznaczenie pradu na zalozonych horyzoncie wstecz
	if (master.step_counter > GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS) {
		sum_of_currents -= currents[current_index];
	}

	sum_of_currents += measured_current;

	currents[current_index] = measured_current;

	current_index = ((++current_index) % GRIPPER_SUM_OF_CURRENTS_NR_OF_ELEMENTS);

	reg_state = next_reg_state;

	//	printf("aa: %d, %d, %d\n",  sum_of_currents, measured_current, kk);
	//printf("aa: %d\n", sum_of_currents);

	switch (reg_state)
	{
		case lib::GRIPPER_START_STATE:

			if (sum_of_currents > GRIPPER_SUM_OF_CURRENTS_MAX_VALUE) {
				next_reg_state = lib::GRIPPER_BLOCKED_STATE;
				gripper_blocked_start_time = master.step_counter;
				//		printf("gripper GRIPPER_BLOCKED_STATE state\n");
			}
			break;

		case lib::GRIPPER_BLOCKED_STATE:
			//	fprintf(stdout,"GRIPPER_BLOCKED_STATE %d\n", measured_current);
			if (((master.step_counter - gripper_blocked_start_time) > GRIPPER_BLOCKED_TIME_PERIOD)
					&& (!(sum_of_currents > GRIPPER_SUM_OF_CURRENTS_MAX_VALUE))) {
				//			printf("gripper GRIPPER_START_STATE state\n");
				next_reg_state = lib::GRIPPER_START_STATE;
			} else {
				position_increment_old = 0;
				position_increment_new = 0;
				delta_eint_old = 0;
				delta_eint = 0;
				step_old_pulse = 0;
				step_new_pulse = 0;
				set_value_very_old = 0;
				set_value_old = 0;
				set_value_old = 0;
				set_value_new = 0;
			}
			break;

		default:
			break;
	}

	prev_reg_state = reg_state;

	return alg_par_status;
}
/*-----------------------------------------------------------------------*/

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
