/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <unistd.h>
#include <cerrno>
#include <ctime>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/mis_fun.h"
#include "base/edp/edp.h"
#include "base/edp/reader.h"
#include "base/edp/HardwareInterface.h"
#include "base/edp/regulator.h"

namespace mrrocpp {
namespace edp {
namespace common {

// regulator


/*-----------------------------------------------------------------------*/
regulator::regulator(uint8_t reg_no, uint8_t reg_par_no, common::motor_driven_effector &_master) :
	master(_master)
{
	// Konstruktor abstrakcyjnego regulatora
	// Inicjuje zmienne, ktore kazdy regulator konkretny musi miec i aktualizowac,
	// niezaleznie od tego czy sa mu niezbedne, czy nie.

	algorithm_no = reg_no; // przeslany numer algorytmu
	current_algorithm_no = reg_no; // numer aktualnie uzywanego algorytmu
	algorithm_parameters_no = reg_par_no; // przeslany numer zestawu parametrow algorytmu
	current_algorithm_parameters_no = reg_par_no; // numer aktualnie uzywanego zestawu parametrow algorytmu

	position_increment_old = 0; // przedostatnio odczytany przyrost polozenie (delta y[k-2] -- mierzone w impulsach)
	position_increment_new = 0; // ostatnio odczytany przyrost polozenie (delta y[k-1] -- mierzone w impulsach)
	step_old_pulse = 0; // poprzednia wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia -- delta r[k-2] -- mierzone w radianach)
	step_old = 0.0; // poprzednia wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia -- delta r[k-2] -- mierzone w radianach)

	step_new = 0; // nastepna wartosc zadana dla jednego kroku regulacji
	// (przyrost wartosci zadanej polozenia -- delta r[k-1] -- mierzone w radianach)
	set_value_new = 0; // wielkosc kroku do realizacji przez HIP (wypelnienie PWM -- u[k])
	set_value_old = 0; // wielkosc kroku do realizacji przez HIP (wypelnienie PWM -- u[k-1])
	set_value_very_old = 0; // wielkosc kroku do realizacji przez HIP (wypelnienie PWM -- u[k-2])

	delta_eint = 0.0; // przyrost calki uchybu
	delta_eint_old = 0.0; // przyrost calki uchybu w poprzednim kroku
	pos_increment_new_sum = 0; // skumulowany przyrost odczytanego polozenia w trakcie realizacji makrokroku
	servo_pos_increment_new_sum = 0;// by Y

	step_new_over_constraint_sum = 0.0;
	previous_abs_position = 0.0;

	meassured_current = 0; // prad zmierzony
	PWM_value = 0; // zadane wypelnienie PWM
}
/*-----------------------------------------------------------------------*/

// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie regulatorow  - stara wersja
/*
 void regulator::constraint_detector(double max_acc_local, double max_vel_local, double max_diff_local, bool debug)
 {
 double step_new_over_constraint_sum_tmp;
 double step_new_tmp = step_new;
 // przyspieszenie i roznica predkosci
 double current_acc, vel_diff;

 //	if (debug) printf("step_new: %f, step_new_over_constraint_sum: %f\n", step_new, step_new_over_constraint_sum);
 step_new += step_new_over_constraint_sum;

 step_new_over_constraint_sum_tmp = step_new;

 // ograniczenie na przekroczenie step_new (aby predkosc biezaca nie odbiegala zby mocno od zadanej,
 // co zmniejsza przeregulowania i oscylacje)
 vel_diff = step_new - step_new_tmp;
 if (vel_diff > max_diff_local) step_new = step_new_tmp + max_diff_local;
 if (vel_diff < -max_diff_local) step_new = step_new_tmp - max_diff_local;

 // sprawdzenie ograniczenia na predkosc (co do predkosci maksymalnej)
 if (step_new > max_vel_local) step_new = max_vel_local;
 if (step_new < -max_vel_local) step_new = -max_vel_local;

 // sprawdzenie ograniczenia na przyspieszenie
 current_acc = step_new - step_old;
 if (current_acc > max_acc_local) step_new = step_old + max_acc_local;
 if (current_acc < -max_acc_local) step_new = step_old - max_acc_local;

 step_new_over_constraint_sum = step_new_over_constraint_sum_tmp - step_new;

 //	if (debug) printf("acc: %f, vel: %f, const_sum: %f\n", current_acc, step_new, step_new_over_constraint_sum);

 step_old = step_new;
 }
 */

// BY Y i S - uwzglednie ograniczen na predkosc i przyspieszenie regulatorow

void regulator::constraint_detector(double max_acc_local, double max_vel_local, bool debug)
{

	double sum_min = 0.0, sum_mid = 0.0, sum_max = 0.0;

	double step_new_beginning = step_new;

	double step_diff = step_old - step_new;
	double abs_step_diff = fabs(step_diff);

	// zliczanie minimalnej, dodatkowej drogi przebytej do czasu zrownania predkosci biezacej z zadana -
	// suma ciagu arytmetycznego dla wariantu minimalnego
	double abs_step_diff_min = abs_step_diff - max_acc_local;
	if (abs_step_diff_min < 0.0)
		abs_step_diff_min = 0.0;

	double first_elem = abs_step_diff_min;
	int step_no = (int) (floor(abs_step_diff_min / max_acc_local));
	double last_elem = abs_step_diff_min - step_no * max_acc_local;

	sum_min = ((first_elem + last_elem) * step_no) / 2;
	if (step_diff < 0)
		sum_min = -sum_min;

	// liczenie sumy sredniej i maksymalnej
	sum_mid = sum_min + step_diff;

	if (step_diff >= 0.0)
		sum_max = sum_mid + step_diff + max_acc_local;
	else if (step_diff < 0.0)
		sum_max = sum_mid + step_diff - max_acc_local;

	// podjecie decyzji o zmianie predkosci
	// jezeli jestesmy dostatecznie blisko zadanej trajektorii
	if ((abs_step_diff <= max_acc_local) && (fabs(step_new_over_constraint_sum) <= max_acc_local)) {
		step_new = step_new_beginning + step_new_over_constraint_sum;
	} else {
		if (step_diff >= 0.0) {
			if (step_new_over_constraint_sum > sum_max) {
				step_new = step_old + max_acc_local;
			} else if (step_new_over_constraint_sum >= sum_mid) {
				step_new = step_old;
			} else if (step_new_over_constraint_sum >= sum_min) {
				step_new = step_old - max_acc_local;
			} else {
				step_new = step_old - max_acc_local;
			}
		} else // 	if (step_diff >= 0.0)
		if (step_new_over_constraint_sum < sum_max) {
			step_new = step_old - max_acc_local;
		} else if (step_new_over_constraint_sum <= sum_mid) {
			step_new = step_old;
		} else if (step_new_over_constraint_sum <= sum_min) {
			step_new = step_old + max_acc_local;
		} else {
			step_new = step_old + max_acc_local;
		}
	}

	// sprawdzenie ograniczenia na predkosc (co do predkosci maksymalnej)
	if (step_new > max_vel_local)
		step_new = max_vel_local;
	if (step_new < -max_vel_local)
		step_new = -max_vel_local;

	step_new_over_constraint_sum = step_new_over_constraint_sum + step_new_beginning - step_new;

	if (debug)
		printf("sn: %f, snb: %f, sum: %f\n", step_new, step_new_beginning, step_new_over_constraint_sum);

	step_old = step_new;
}

double regulator::get_set_value(void) const
{
	// odczytanie aktualnej wartosci zadanej  - metoda konkretna
	return set_value_new;
}

void regulator::insert_new_step(double ns)
{
	// wstawienie nowej wartosci zadanej - metoda konkretna
	step_new = ns;
}

void regulator::insert_meassured_current(int meassured_current_l)
{
	// wstawienie wartosci zmierzonej pradu
	meassured_current = meassured_current_l;
}

double regulator::return_new_step(void) const
{
	// wstawienie nowej wartosci zadanej - metoda konkretna
	return step_new;
}

void regulator::insert_new_pos_increment(double inc)
{
	// wstawienie nowej wartosci odczytanej przyrostu polozenia - metoda konkretna
	// Do przepisywania zrealizowanego polozenia z HI do regulatora
	position_increment_new = inc;
}

double regulator::get_position_inc(int tryb)
{ // by Y: 0 dla servo i 1 dla paczki dla edp;
	// odczytanie zrealizowanego przyrostu polozenia w makrokroku - metoda konkretna
	double pins;
	if (tryb == 1) {
		pins = pos_increment_new_sum;
		pos_increment_new_sum = 0.0;
	} else if (tryb == 0) {
		pins = servo_pos_increment_new_sum;
		servo_pos_increment_new_sum = 0.0;
	} else {
		pins = false;
	}
	return pins;
}

int regulator::get_meassured_current(void) const
{
	// odczytanie rzeczywistego pradu - metoda konkretna
	return meassured_current;
}

int regulator::get_PWM_value(void) const
{
	// odczytanie zadanego wypelnienia PWM - metoda abstrakcyjna
	return PWM_value;
}

// do odczytu stanu regulatora (w szczegolnosci regulatora chwytaka)
int regulator::get_reg_state(void) const
{
	// odczytanie zadanego wypelnienia PWM - metoda abstrakcyjna
	return reg_state;
}

int regulator::get_actual_inc(void) const
{
	// odczytanie rzeczywistego przyrostu polozenia w pojedynczym kroku
	return (int) position_increment_new;
}

// double get_desired_inc ( int axe_nr );


void regulator::insert_algorithm_no(uint8_t new_number)
{
	// wpisanie nowego numeru algorytmu regulacji
	algorithm_no = new_number;
}

uint8_t regulator::get_algorithm_no(void) const
{
	// odczytanie aktualnie uzywanego numeru algorytmu regulacji
	return current_algorithm_no;
}

void regulator::insert_algorithm_parameters_no(uint8_t new_number)
{
	// wpisanie nowego numeru zestawu parametrow algorytmu regulacji
	algorithm_parameters_no = new_number;
}

uint8_t regulator::get_algorithm_parameters_no(void) const
{
	// wpisanie nowego numeru zestawu parametrow algorytmu regulacji
	return current_algorithm_parameters_no;
}

void regulator::clear_regulator()

{
	// zerowanie wszystkich zmiennych regulatora
	position_increment_old = 0.0;
	position_increment_new = 0.0;
	pos_increment_new_sum = 0.0;
	servo_pos_increment_new_sum = 0.0;
	step_old_pulse = 0.0;
	step_new = 0.0;
	set_value_new = 0.0;
	set_value_old = 0.0;
	set_value_very_old = 0.0;
	delta_eint = 0.0;
	delta_eint_old = 0.0;
}

/*-----------------------------------------------------------------------*/
NL_regulator::NL_regulator(uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	regulator(reg_no, reg_par_no, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.

	a = aa; // parametr regulatora
	b0 = bb0; // parametr regulatora
	b1 = bb1; // parametr regulatora
	k_feedforward = k_ff; // wspolczynnik wzmocnienia w petli "feedforward"

	EPS = 1.0e-10;
	MAX_PWM = 190; // Maksymalne wypelnienie PWM dla robota IRp-6 na torze
	integrator_off = 6;
	counter = 0;

	meassured_current = 0;
}
/*-----------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp

