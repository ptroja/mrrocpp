/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

// Klasa edp_irp6ot_effector.
//#include "robot/irp6ot_tfg/edp_irp6ot_tfg_effector.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
//#include "base/edp/irp6_on_track/hi_irp6ot.h"
// Klasa servo_buffer.


#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6ot_m/regulator_irp6ot_m.h"
#include "robot/irp6ot_m/edp_irp6ot_m_effector.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_m {

/*-----------------------------------------------------------------------*/
NL_regulator_1_irp6ot::NL_regulator_1_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
NL_regulator_2_irp6ot::NL_regulator_2_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
NL_regulator_3_irp6ot::NL_regulator_3_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
NL_regulator_4_irp6ot::NL_regulator_4_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
NL_regulator_5_irp6ot::NL_regulator_5_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
	first = true;
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
NL_regulator_6_irp6ot::NL_regulator_6_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
NL_regulator_7_irp6ot::NL_regulator_7_irp6ot(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_1_irp6ot::compute_set_value(void)
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

	//   struct timespec step_time;

	alg_par_status = common::ALGORITHM_AND_PARAMETERS_OK;

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new * AXIS_0_TO_5_INC_PER_REVOLUTION / (2 * M_PI);
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


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005 * (step_new_pulse - position_increment_new) - 0.995 * (step_old_pulse
			- position_increment_old);

	// if (fabs(step_new_pulse) > 70.0) {
	//  cprintf("snp = %lf   pin = %lf\n",step_new_pulse, position_increment_new);
	// }

	// if (fabs(delta_eint) > 50.0) {
	//  cprintf("%4.0lf ",delta_eint);
	// }

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
						a = 0.4152;
						b0 = 0.9017 * 1.5;
						b1 = 0.7701 * 1.5;
						k_feedforward = 0.35;
						break;
					case 1: // zestaw parametrow nr 1
						current_algorithm_parameters_no = algorithm_parameters_no;
						current_algorithm_no = algorithm_no;
						a = 0.4152;
						b0 = 0.9017 * 1.0;
						b1 = 0.7701 * 1.0;
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
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}
	}

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;
			if ((fabs(set_value_new)) < 0.1)
				counter++;
			else
				counter = 0;

			if (fabs(step_new) < EPS && fabs(position_increment_new) < EPS && (counter > integrator_off)) {
				set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
						- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
			}
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
			break;
		default: // w tym miejscu nie powinien wystapic blad zwiazany z
			// nieistniejacym numerem algorytmu
			set_value_new = 0; // zerowe nowe sterowanie
			break;
	}

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[0] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[0] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[0] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[0] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[0] = measured_current;
	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

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
uint8_t NL_regulator_2_irp6ot::compute_set_value(void)
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


	uint8_t alg_par_status; // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow


	alg_par_status = common::ALGORITHM_AND_PARAMETERS_OK;

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new * AXIS_0_TO_5_INC_PER_REVOLUTION / (2 * M_PI);

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


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005 * (step_new_pulse - position_increment_new) - 0.995 * (step_old_pulse
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
						b0 = 2.3100 * 1.5;
						b1 = 2.0312 * 1.5;
						k_feedforward = 0.35;
						break;
					case 1: // zestaw parametrow nr 1
						current_algorithm_parameters_no = algorithm_parameters_no;
						current_algorithm_no = algorithm_no;
						a = 0.3079;
						b0 = 2.3100 * 2.0;
						b1 = 2.0312 * 2.0;
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
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}
	}

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
			break;
		default: // w tym miejscu nie powinien wystapic blad zwiazany z
			// nieistniejacym numerem algorytmu
			set_value_new = 0; // zerowe nowe sterowanie
			break;
	}

	// printf("banana1: %f\n", set_value_new);

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[1] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[1] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[1] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[1] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[1] = measured_current;
	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

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
uint8_t NL_regulator_3_irp6ot::compute_set_value(void)
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

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new * AXIS_0_TO_5_INC_PER_REVOLUTION / (2 * M_PI);
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


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005 * (step_new_pulse - position_increment_new) - 0.995 * (step_old_pulse
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
						a = 0.4152;
						b0 = 1.2500 * 1.5;
						b1 = 1.0998 * 1.5;
						k_feedforward = 0.35;
						break;
					case 1: // zestaw parametrow nr 1
						current_algorithm_parameters_no = algorithm_parameters_no;
						current_algorithm_no = algorithm_no;
						a = 0.4152;
						b0 = 1.2500 * 2.5;
						b1 = 1.0998 * 2.5;
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
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}
	}

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
			break;
		default: // w tym miejscu nie powinien wystapic blad zwiazany z
			// nieistniejacym numerem algorytmu
			set_value_new = 0; // zerowe nowe sterowanie
			break;
	}

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[2] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[2] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[2] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[2] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[2] = measured_current;
	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

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
uint8_t NL_regulator_4_irp6ot::compute_set_value(void)
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

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new * AXIS_0_TO_5_INC_PER_REVOLUTION / (2 * M_PI);

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


	servo_pos_increment_new_sum += position_increment_new;// by Y
	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005 * (step_new_pulse - position_increment_new) - 0.995 * (step_old_pulse
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
						b0 = 1.0942 * 1.0;
						b1 = 0.9166 * 1.0;
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

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
			break;
		default: // w tym miejscu nie powinien wystapic blad zwiazany z
			// nieistniejacym numerem algorytmu
			set_value_new = 0; // zerowe nowe sterowanie
			break;
	}

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[3] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[3] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[3] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[3] = (float) (step_new_pulse - position_increment_new);
		// master.rb_obj->step_data.uchyb[3]=(float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[3] = measured_current;

	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

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
uint8_t NL_regulator_5_irp6ot::compute_set_value(void)
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

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new * AXIS_0_TO_5_INC_PER_REVOLUTION / (2 * M_PI);

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


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005 * (step_new_pulse - position_increment_new) - 0.995 * (step_old_pulse
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
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}
	}

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
			break;
		default: // w tym miejscu nie powinien wystapic blad zwiazany z
			// nieistniejacym numerem algorytmu
			set_value_new = 0; // zerowe nowe sterowanie
			break;
	}

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[4] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[4] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[4] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[4] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[4] = measured_current;
	}

	// ograniczenie na sterowanie
	if (set_value_new > MAX_PWM)
		set_value_new = MAX_PWM;
	if (set_value_new < -MAX_PWM)
		set_value_new = -MAX_PWM;

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
uint8_t NL_regulator_6_irp6ot::compute_set_value(void)
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

	// przeliczenie radianow na impulsy
	step_new_pulse = step_new * AXIS_0_TO_5_INC_PER_REVOLUTION / (2 * M_PI);

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


	// kumulacja przyrostu polozenia w tym makrokroku
	pos_increment_new_sum += position_increment_new;
	servo_pos_increment_new_sum += position_increment_new;// by Y

	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.005 * (step_new_pulse - position_increment_new) - 0.995 * (step_old_pulse
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
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}
	}

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old;
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
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

	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[5] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[5] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[5] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[5] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[5] = measured_current;
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

	return alg_par_status;

}
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_7_irp6ot::compute_set_value(void)
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
	uint8_t alg_par_status; // okresla prawidlowosc numeru algorytmu regulacji
	// i zestawu jego parametrow

	alg_par_status = common::ALGORITHM_AND_PARAMETERS_OK;

	// double root_position_increment_new=position_increment_new;


	// przeliczenie radianow na impulsy
	// step_new_pulse = step_new*AXIS_6_INC_PER_REVOLUTION/(2*M_PI); // ORIGINAL
	step_new_pulse = step_new * AXIS_6_INC_PER_REVOLUTION / (2 * M_PI);
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
	 if (fabs(position_increment_new) > common::MAX_INC)
	 position_increment_new = position_increment_old;
	 */

	// kumulacja przyrostu polozenia w tym makrokroku // ORIGINAL
	// pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO;
	// servo_pos_increment_new_sum += position_increment_new*POSTUMENT_TO_TRACK_RATIO; // by Y

	// kumulacja przyrostu polozenia w tym makrokroku
	//pos_increment_new_sum += root_position_increment_new;
	//servo_pos_increment_new_sum += root_position_increment_new;// by Y

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

	a = 0.3;
	b0 = 1.364 * POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //4
	b1 = 1.264 * POSTUMENT_TO_TRACK_VOLTAGE_RATIO; //1.364//4

	switch (algorithm_no)
	{
		case 0: // algorytm nr 0
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = ((1 + a) * set_value_old - a * set_value_very_old + b0 * delta_eint - b1 * delta_eint_old);
			break;
		case 1: // algorytm nr 1
			// obliczenie nowej wartosci wypelnienia PWM algorytm PD + I
			set_value_new = (1 + a) * set_value_old - a * set_value_very_old + b0 * (step_new_pulse
					- position_increment_new) - b1 * (step_old_pulse - position_increment_old);
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


	// scope-locked reader data update
	{
		boost::mutex::scoped_lock lock(master.rb_obj->reader_mutex);

		master.rb_obj->step_data.desired_inc[6] = (float) step_new_pulse; // pozycja osi 0
		master.rb_obj->step_data.current_inc[6] = (short int) position_increment_new;
		master.rb_obj->step_data.pwm[6] = (float) set_value_new;
		master.rb_obj->step_data.uchyb[6] = (float) (step_new_pulse - position_increment_new);
		master.rb_obj->step_data.measured_current[6] = measured_current;
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

	return alg_par_status;

}
/*-----------------------------------------------------------------------*/

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
