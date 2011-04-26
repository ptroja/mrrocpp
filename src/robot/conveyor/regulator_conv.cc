// ------------------------------------------------------------------------
//                                       edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota IRp-6 na torze - metody: class edp_irp6s_and_conv_robot
//
// Ostatnia modyfikacja: styczen 2005
// -------------------------------------------------------------------------

#include <cstdio>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/edp/edp_typedefs.h"
#include "base/edp/reader.h"
#include "robot/conveyor/const_conveyor.h"
#include "robot/conveyor/regulator_conv.h"

//#include "base/edp/edp_e_motor_driven.h"
#include "robot/conveyor/edp_conveyor_effector.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

// uint64_t kk;	// numer pomiaru od momentu startu pomiarow


/*-----------------------------------------------------------------------*/
NL_regulator_1_conv::NL_regulator_1_conv(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}
/*-----------------------------------------------------------------------*/

// tasmociag


/*-----------------------------------------------------------------------*/
uint8_t NL_regulator_1_conv::compute_set_value(void)
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
	step_new_pulse = step_new * INC_PER_REVOLUTION / (2 * M_PI);
	//position_increment_new=position_increment_new/AXE_0_TO_5_POSTUMENT_TO_TRACK_RATIO;

	/*
	 if (!aaa)
	 if ( (fabs(step_new_pulse) < 0.0001) && (fabs(position_increment_new ) > 1) ) {
	 aaa++;
	 }
	 */
	// if (aaa > 0 && aaa < 30 ) {
	//  cprintf("O1: svn=%4.0lf svo=%4.0lf svvo=%4.0lf de=%4.0lf deo=%4.0lf snp=%4.0lf pin=%4.0lf pio=%4.0lf\n",set_value_new,set_value_old,
	//  set_value_very_old, delta_eint, delta_eint_old, step_new_pulse,position_increment_new,position_increment_old);
	//  cprintf("O1: snp=%4.0lf pin=%4.0lf pio=%4.0lf L=%4x U=%4x\n", step_new_pulse,position_increment_new,position_increment_old,md.robot_status[0].adr_224,md.robot_status[0].adr_226);
	//  aaa++;
	//  if (aaa == 9) aaa=0;
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
	// pos_increment_new_sum += root_position_increment_new;
	// servo_pos_increment_new_sum += root_position_increment_new;// by Y


	// Przyrost calki uchybu
	delta_eint = delta_eint_old + 1.008 * (step_new_pulse - position_increment_new) - 0.992 * (step_old_pulse
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
				; // end: switch (algorithm_parameters_no)
				break;
			default: // blad - nie ma takiego algorytmu
				// => przywrocic stary algorytm i j stary zestaw parametrow
				algorithm_no = current_algorithm_no;
				algorithm_parameters_no = current_algorithm_parameters_no;
				alg_par_status = common::UNIDENTIFIED_ALGORITHM_NO;
				break;
		}; // end: switch (algorithm_no)
	}

	a = 0.548946716233;
	b0 = 1.576266 * CONVEYOR35V_TO_CONVEYOR_VOLTAGE_RATIO; //9.244959545156;
	b1 = 1.468599 * CONVEYOR35V_TO_CONVEYOR_VOLTAGE_RATIO; //8.613484947882;


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

} // namespace common
} // namespace edp
} // namespace mrrocpp
