/*!
 * @file
 * @brief File containing the methods of the model_with_wrist class.
 *
 * The model_with_wrist kinematic model utilizes six out of seven IRP-6ot DOF - the track is treated as a passive one.
 *
 * @author tkornuta
 * @date 24.02.2007
 *
 * @ingroup KINEMATICS IRP6OT_KINEMATICS irp6ot_m
 */


#include <cstdio>
#include <cmath>

#include "base/lib/com_buf.h"
#include "robot/irp6ot_m/kinematic_model_irp6ot_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6ot {

model_with_wrist::model_with_wrist(int _number_of_servos) :
	number_of_servos(_number_of_servos)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to  kinematic model with active wrist");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();

	// Wykonywac przeliczenia zwiazane z narzedziami.
	attached_tool_computations = true;
}

void model_with_wrist::set_kinematic_parameters(void)
{
	/* -----------------------------------------------------------------------
	 Numery osi:
	 0 - tor jezdny
	 1 - kolumna obrotowa: os FI
	 2 - ramie dolne:     os TETA
	 3 - ramie gorne:    os ALFA
	 4 - pochylnie kisci: os T
	 5 - obrot kisci:    os V
	 7 - obrot kisci:    os N
	 8 - chwytak
	 ------------------------------------------------------------------------- */

	/* -----------------------------------------------------------------------
	 Dlugosci czlonow robota [m]
	 ------------------------------------------------------------------------- */
	d1 = 0.7;
	a2 = 0.455;
	a3 = 0.67;
	d5 = 0.19;
	d6 = 0.095;
	d7 = 0.20;

	/* -----------------------------------------------------------------------
	 Ustawienie standardowego narzedzia [m]
	 ------------------------------------------------------------------------- */
	// Ustawienie srodka szczek chwytaka.
	tool.set_translation_vector(0, 0, 0.25);

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przelozenie przekladni kata obrotu walu silnika napedowego toru.
	 *  gear_0 [rad/m]    = 1000[mm/m]*K_p*2*pi[rad/obr]/S_k[mm]*K_0
	 *  S_k = 8mm        - skok sruby kulowej
	 *  K_p = 2          - przelozenie przekladni pasowej
	 *  K_0               - 1 lub -1 w zaleznosci od tego czy dodatni kierunek ruchu
	 *                     walu silnika jest zgodny z dodatnim kierunkiem ruchu osi
	 ------------------------------------------------------------------------- */
	double S_k = 8;
	double K_p = 2;
	double K_0 = 1;
	gear[0] = (1000* K_p * 2*M_PI / S_k) * K_0;
	theta[0] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przelozenie przekladni kata obrotu walu silnika napedowego kolumny.
	 *  gear_1    =  H_1 * K_1
	 *  H_1        - przelozenie przekladni harmonicznej kolumny
	 *  K_1        - 1 lub -1 w zaleznosci od tego czy dodatni kierunek ruchu
	 *              walu silnika jest zgodny z dodatnim kierunkiem ruchu osi
	 ------------------------------------------------------------------------- */
	double H_1 = 158;
	double K_1 = -1;
	gear[1] = (H_1 * K_1);
	theta[1] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnikow - przelozenie przekladni kata obrotu walu silnika napedowego ramienia dolnego.
	 *  gear_2    =  (2 * M_PI / S_r) * k2
	 gdzie
	 * S_r		- Skok sruby kulowej ramion w [mm]
	 * k2		- kierunek obrotu osi nr 2 wzgledem dodatniego kierunku
	 * theta2_0		- poczatkowa wartosc
	 parametry przekladni srubowo-tocznych dla ramion wyliczone z wzorow
	 * sl123	= l1**2 + l2**2 + l3**2
	 * mi2		= 2*l1*(l2*cl + l3*sl)
	 * ni2		= 2*l1*(l3*cl - l2*sl)
	 ------------------------------------------------------------------------- */
	sl123 = 7.789525e+04;
	mi2 = 6.090255e+04;
	ni2 = -2.934668e+04;

	double S_r = 5;
	double K_2 = 1;
	gear[2] = (2*M_PI / S_r) * K_2;
	theta[2] = 2.203374e+02; // l02

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przelozenie przekladni kata obrotu walu silnika napedowego ramienia gornego.
	 *  gear_3    =  (2 * M_PI / S_r) * k3
	 gdzie
	 * S_r 		- Skok sruby kulowej ramion w [mm]
	 * k3		- kierunek obrotu osi nr 2 wzgledem dodatniego kierunku
	 * theta3_0	- poczatkowa wartosc
	 parametry przekladni srubowo-tocznych dla ramion wyliczone z wzorow
	 * mi3		= -2 * l1 * l2
	 * ni3		= -2 * l1 * l3
	 ------------------------------------------------------------------------- */
	mi3 = -4.410000e+04;
	ni3 = -5.124000e+04;

	double K_3 = 1;
	gear[3] = (2*M_PI / S_r) * K_3;
	theta[3] = 1.838348e+02; // l03

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przekladni kata obrotu walu silnika napedowego pochylenie kisci T.
	 *  gear_4 = H_4*K_4
	 gdzie
	 *  H_4        przelozenie przekladni harmonicznej osi nr 4
	 *  K_4        = -1 lub 1 w zaleznosci od tego czy dodatnie inkrementy
	 *              odpowiadaja wzrostowi lub maleniu kata t234
	 ------------------------------------------------------------------------- */
	double H_4 = 128;
	double K_4 = -1;
	gear[4] = H_4 * K_4;
	theta[4] = 1.570796e+00; // T2340

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przekladni kata obrotu walu silnika napedowego pochylenie kisci V.
	 * gear_5 = H_5*S_5*K_5;
	 gdzie
	 *  H_5        = przelozenie przekladni harmonicznej osi nr 5
	 *  S_5        = przelozenie przekladni stozkowej
	 *  K_5        = 1 lub -1 zaleznosci od zgodnosci kierunkow wzrostu
	 *				kata t5 i inkrementow
	 ------------------------------------------------------------------------- */
	double H_5 = 128;
	double S_5 = 0.6;
	double K_5 = -1;
	gear[5] = H_5 * S_5 * K_5;
	theta[5] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przekladni kata obrotu walu silnika napedowego pochylenie kisci N.
	 * gear_6 = H_6*K_6;
	 gdzie
	 *  H_6        = przelozenie przekladni harmonicznej osi nr 6
	 *  K_6        = 1 lub -1 zaleznosci od zgodnosci kierunkow wzrostu
	 *				kata t6 i inkrementow
	 ------------------------------------------------------------------------- */
	double H_6 = 288.8845;
	double K_6 = 1;
	gear[6] = H_6 * K_6;
	theta[6] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Wspolczynniki uzywane przy obliczeniach zwarcia/rozwarcia szczek:
	 Obliczenia wartosci wspolrzednych wewnetrznych na podstawie odczytow enkoderow silnikow
	 * joint[7] = dir_a_7 * motor[7]^2 + dir_b_7 * motor[7] + dir_c_7
	 uzywane wspolczynniki
	 * dir_a_7, dir_b_7, dir_c_7

	 Obliczenia wartosci polozen silnikow na podstawie wspolrzednych wewnetrznych
	 * motor[7] = inv_a_7 * sqrt(inv_b_7 + inv_c_7 * joint[7]) + inv_d_7
	 uzywane wspolczynniki
	 * inv_a_7, inv_b_7, inv_c_7, inv_d_7
	 ------------------------------------------------------------------------- */
	dir_a_7 = -0.00000000283130;
	dir_b_7 = 0.00001451910074;
	dir_c_7 = 0.074;
	inv_a_7 = 0.3531946456e-5;
	inv_b_7 = 0.2622172716e19;
	inv_c_7 = -0.2831300000e20;
	inv_d_7 = -2564.034320;
	gear[7] = 0.0;
	theta[7] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji - odczyty z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_motor_position[0] = 0; // tor [m]
	synchro_motor_position[1] = -13.819; // kolumna [rad]
	//synchro_motor_position[1]= -7.5;		// kolumna [rad]
	synchro_motor_position[2] = -5.012; // ramie d. [rad]
	synchro_motor_position[3] = -4.219; // ramie g. [rad]
	synchro_motor_position[4] = 155.997; // kisc T [rad]
	synchro_motor_position[5] = 476.5; // kisc V [rad] poprawne front position w motorach (6 os z kolei oznaczona jako 5) + 320.25
	synchro_motor_position[6] = 769.7; // kisc N [rad]
	synchro_motor_position[7] = 4830; // chwytak [-]

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji we wspolrzednych wewnetrznych - obliczone na podstawie z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_joint_position[0] = synchro_motor_position[0] - gear[0] * theta[0];
	synchro_joint_position[1] = synchro_motor_position[1] - gear[1] * theta[1];
	synchro_joint_position[2] = synchro_motor_position[2] - gear[2] * theta[2];
	synchro_joint_position[3] = synchro_motor_position[3] - gear[3] * theta[3];
	synchro_joint_position[4] = synchro_motor_position[4] - gear[4] * theta[4];
	synchro_joint_position[5] = synchro_motor_position[5] - gear[5] * theta[5] - synchro_motor_position[4];
	synchro_joint_position[6] = synchro_motor_position[6] - gear[6] * theta[6];
	synchro_joint_position[7] = synchro_motor_position[7] - gear[7] * theta[7];

	/* -----------------------------------------------------------------------
	 Zakresy ruchu walow silnikow w radianach.
	 ------------------------------------------------------------------------- */
	lower_limit_axis[0] = -200;
	lower_limit_axis[1] = -470;
	lower_limit_axis[2] = -110;
	lower_limit_axis[3] = -80;
	lower_limit_axis[4] = -70;
	lower_limit_axis[5] = -80;
	lower_limit_axis[6] = -1000;
	lower_limit_axis[7] = -2000;

	upper_limit_axis[0] = 1900;
	upper_limit_axis[1] = 450;
	upper_limit_axis[2] = 100;
	upper_limit_axis[3] = 100;
	upper_limit_axis[4] = 380;
	upper_limit_axis[5] = 490;
	upper_limit_axis[6] = 3000;
	upper_limit_axis[7] = 5000;

	/* -----------------------------------------------------------------------
	 Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach).
	 ------------------------------------------------------------------------- */
	lower_limit_joint[0] = -0.125; // [m]
	lower_limit_joint[1] = -170.0 * M_PI / 180.0;
	lower_limit_joint[2] = -130.0 * M_PI / 180.0;
	lower_limit_joint[3] = -25.0 * M_PI / 180.0;
	lower_limit_joint[4] = -90.0 * M_PI / 180.0;
	lower_limit_joint[5] = -10.0; // -M_PI
	lower_limit_joint[6] = -2.88;
	lower_limit_joint[7] = 0.053;

	upper_limit_joint[0] = 1.21; // [m];
	upper_limit_joint[1] = 170.0 * M_PI / 180.0; // [rad]
	upper_limit_joint[2] = -50.0 * M_PI / 180.0;
	upper_limit_joint[3] = 40.0 * M_PI / 180.0;
	upper_limit_joint[4] = 92*M_PI / 180.0;
	upper_limit_joint[5] = 10.0; //M_PI
	upper_limit_joint[6] = 2.93;
	upper_limit_joint[7] = 0.091;

}// end: set_kinematic_parameters


void model_with_wrist::check_motor_position(const lib::MotorArray & motor_position)
{

	if (motor_position[0] < lower_limit_axis[0]) // Kat f1 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_0);
	else if (motor_position[0] > upper_limit_axis[0]) // Kat f1 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_0);

	if (motor_position[1] < lower_limit_axis[1]) // Kat f2 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_1);
	else if (motor_position[1] > upper_limit_axis[1]) // Kat f2 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_1);

	if (motor_position[2] < lower_limit_axis[2]) // Kat f3 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_2);
	else if (motor_position[2] > upper_limit_axis[2]) // Kat f3 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_2);

	if (motor_position[3] < lower_limit_axis[3]) // Kat f4 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_3);
	else if (motor_position[3] > upper_limit_axis[3]) // Kat f4 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_3);

	if (motor_position[4] < lower_limit_axis[4]) // Kat f5 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_4);
	else if (motor_position[4] > upper_limit_axis[4]) // Kat f5 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_4);

	if (motor_position[5] < lower_limit_axis[5]) // Kat f6 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_5);
	else if (motor_position[5] > upper_limit_axis[5]) // Kat f6 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_5);

	if (motor_position[6] < lower_limit_axis[6]) // Kat f7 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_6);
	else if (motor_position[6] > upper_limit_axis[6]) // Kat f7 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_6);

	if (number_of_servos > 7) {
		if (motor_position[7] < lower_limit_axis[7]) // Kat f8 mniejszy od minimalnego
			throw NonFatal_error_2(BEYOND_LOWER_LIMIT_AXIS_7);
		else if (motor_position[7] > upper_limit_axis[7]) // Kat f8 wiekszy od maksymalnego
			throw NonFatal_error_2(BEYOND_UPPER_LIMIT_AXIS_7);
	}

}//: check_motor_position


void model_with_wrist::check_joints(const lib::JointArray & q)
{

	if (isnan(q[0]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_D0);

	if (q[0] < lower_limit_joint[0]) // kat q1 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_D0_LIMIT);
	else if (q[0] > upper_limit_joint[0]) // kat q1 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_D0_LIMIT);

	if (isnan(q[1]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA1);
	if (q[1] < lower_limit_joint[1]) // dlugosc silownika mniejsza od minimalnej
		throw NonFatal_error_2(BEYOND_LOWER_THETA1_LIMIT);

	if (q[1] > upper_limit_joint[1]) // dlugosc silownika wieksza od maksymalnej
		throw NonFatal_error_2(BEYOND_UPPER_THETA1_LIMIT);

	if (isnan(q[2]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA2);
	if (q[2] < lower_limit_joint[2]) // dlugosc silownika mniejsza od minimalnej
		throw NonFatal_error_2(BEYOND_LOWER_THETA2_LIMIT);

	if (q[2] > upper_limit_joint[2]) // dlugosc silownika wieksza od maksymalnej
		throw NonFatal_error_2(BEYOND_UPPER_THETA2_LIMIT);

	if (isnan(q[3]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA3);
	if (q[3] < lower_limit_joint[3]) // kat q3 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_THETA3_LIMIT);

	if (q[3] > upper_limit_joint[3])
		throw NonFatal_error_2(BEYOND_UPPER_THETA3_LIMIT);

	if (isnan(q[4]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA4);
	if (q[4] < lower_limit_joint[4]) // kat q4 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_THETA4_LIMIT);

	if (q[4] > upper_limit_joint[4]) // kat q4 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_THETA4_LIMIT);

	if (isnan(q[5]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA5);
	if (q[5] < lower_limit_joint[5]) // kat q5 mniejszy od minimalnego
		throw NonFatal_error_2(BEYOND_LOWER_THETA5_LIMIT);

	if (q[5] > upper_limit_joint[5]) // kat q5 wiekszy od maksymalnego
		throw NonFatal_error_2(BEYOND_UPPER_THETA5_LIMIT);

	if (isnan(q[6]))
		throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA6);
	if (q[6] < lower_limit_joint[6]) // 6 st. swobody
		throw NonFatal_error_2(BEYOND_LOWER_THETA6_LIMIT);

	if (q[6] > upper_limit_joint[6]) // 6 st. swobody
		throw NonFatal_error_2(BEYOND_UPPER_THETA6_LIMIT);

	//***szczeki chwytaka***
	if (number_of_servos > 7) {
		if (isnan(q[7]))
			throw NonFatal_error_2(NOT_A_NUMBER_JOINT_VALUE_THETA7);
		if (q[7] < lower_limit_joint[7]) // 7 st. swobody
			throw NonFatal_error_2(BEYOND_LOWER_THETA7_LIMIT);

		if (q[7] > upper_limit_joint[7]) // 7 st. swobody
			throw NonFatal_error_2(BEYOND_UPPER_THETA7_LIMIT);
	}
}//: check_joints


void model_with_wrist::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{

	// zmienne pomocnicze
	double c, d, l;
	double sinus, cosinus;
	double M2, M3;

	// Sprawdzenie ograniczen na poloenia katowe walow silnikow
	check_motor_position(local_current_motor_pos);

	// Przelicznik polozenia walu silnika napedowego toru w radianach
	// na przesuniecie toru (wspolrzedna wewnetrzna) w metrach
	local_current_joints[0] = (local_current_motor_pos[0] - synchro_motor_position[0]) / gear[0];

	// Przelicznik polozenia walu silnika napedowego kolumny w radianach
	// na kat obrotu kolumny (wspolrzedna wewnetrzna) w radianach
	local_current_joints[1] = (local_current_motor_pos[1] - synchro_motor_position[1]) / gear[1] + theta[1];

	// Przelicznik polozenia walu silnika napedowego ramienia dolnego w radianach
	// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
	l = (local_current_motor_pos[2] - synchro_motor_position[2]) / gear[2] + theta[2];
	M2 = mi2 * mi2 + ni2 * ni2;
	c = l * l - sl123;
	d = sqrt(M2 - c * c);
	cosinus = (mi2 * c - ni2 * d) / M2;
	sinus = -(ni2 * c + mi2 * d) / M2;
	local_current_joints[2] = atan2(sinus, cosinus);

	// Przelicznik polozenia walu silnika napedowego ramienia gornego w radianach
	// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
	l = (local_current_motor_pos[3] - synchro_motor_position[3]) / gear[3] + theta[3];
	M3 = mi3 * mi3 + ni3 * ni3;
	c = l * l - sl123;
	d = sqrt(M3 - c * c);
	cosinus = (mi3 * c - ni3 * d) / M3;
	sinus = -(ni3 * c + mi3 * d) / M3;
	local_current_joints[3] = atan2(sinus, cosinus);

	// Przelicznik polozenia walu silnika napedowego obrotu kisci T w radianach
	// na kat pochylenia kisci (wspolrzedna wewnetrzna) w radianach
	local_current_joints[4] = (local_current_motor_pos[4] - synchro_motor_position[4]) / gear[4];

	// Przelicznik polozenia walu silnika napedowego obrotu kisci V w radianach
	// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
	local_current_joints[5] = (local_current_motor_pos[5] - synchro_motor_position[5] - (local_current_motor_pos[4]
			- synchro_motor_position[4])) / gear[5] + theta[5];

	// Przelicznik polozenia walu silnika napedowego obrotu kisci N w radianach
	// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
	local_current_joints[6] = (local_current_motor_pos[6] - synchro_motor_position[6]) / gear[6] + theta[6];

	if (number_of_servos > 7) {
	// Przelicznik polozenia walu silnika szczek na ich zacisniecie
	local_current_joints[7] = dir_a_7 * (local_current_motor_pos[7] * local_current_motor_pos[7]) - dir_b_7
			* local_current_motor_pos[7] + dir_c_7;
	}
	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	check_joints(local_current_joints);

}//: mp2i_transform


void model_with_wrist::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	// Niejednoznacznosc polozenia dla 4-tej osi (obrot kisci < 180).
	double joint_4_revolution = M_PI;
	// Niejednoznacznosc polozenia dla 5-tej osi (obrot kisci > 360).
	double axis_5_revolution = 2*M_PI;

	// Sprawdzenie wartosci wspolrzednych wewnetrznych.
	check_joints(local_desired_joints);

	// Obliczanie kata obrotu walu silnika napedowego toru
	local_desired_motor_pos_new[0] = gear[0] * local_desired_joints[0] + synchro_joint_position[0];

	// Obliczanie kata obrotu walu silnika napedowego kolumny
	local_desired_motor_pos_new[1] = gear[1] * local_desired_joints[1] + synchro_joint_position[1];

	// Obliczanie kata obrotu walu silnika napedowego ramienia dolnego
	local_desired_motor_pos_new[2] = gear[2] * sqrt(sl123 + mi2 * cos(local_desired_joints[2]) + ni2
			* sin(-local_desired_joints[2])) + synchro_joint_position[2];

	// Obliczanie kata obrotu walu silnika napedowego ramienia gornego
	local_desired_motor_pos_new[3] = gear[3] * sqrt(sl123 + mi3 * cos(local_desired_joints[3]) + ni3
			* sin(-local_desired_joints[3])) + synchro_joint_position[3];

	// Obliczanie kata obrotu walu silnika napedowego obotu kisci T
	// jesli jest mniejsze od -pi/2
	double tmp_local_desired_joints4 = local_desired_joints[4];
	if ( tmp_local_desired_joints4 < lower_limit_joint[4])
		 tmp_local_desired_joints4 += joint_4_revolution;
	local_desired_motor_pos_new[4] = gear[4] * ( tmp_local_desired_joints4 + theta[4]) + synchro_joint_position[4];

	// Obliczanie kata obrotu walu silnika napedowego obrotu kisci V
	local_desired_motor_pos_new[5] = gear[5] * local_desired_joints[5] + synchro_joint_position[5]
			+ local_desired_motor_pos_new[4];

	// Ograniczenie na obrot.
	while (local_desired_motor_pos_new[5] < lower_limit_axis[5])
		local_desired_motor_pos_new[5] += axis_5_revolution;
	while (local_desired_motor_pos_new[5] > upper_limit_axis[5])
		local_desired_motor_pos_new[5] -= axis_5_revolution;

	// Obliczanie kata obrotu walu silnika napedowego obrotu kisci N
	local_desired_motor_pos_new[6] = gear[6] * local_desired_joints[6] + synchro_joint_position[6];

	if (number_of_servos > 7) {
	// Obliczenie kata obrotu walu silnika napedowego chwytaka.
	local_desired_motor_pos_new[7] = inv_a_7 * sqrt(inv_b_7 + inv_c_7 * local_desired_joints[7]) + inv_d_7;
	}

	// Sprawdzenie obliczonych wartosci.
	check_motor_position(local_desired_motor_pos_new);

}//: i2mp_transform


void model_with_wrist::direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{

	// Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
	check_joints(local_current_joints);

	// Parametry pomocnicze - przeliczenie zmiennych.
	double d0 = local_current_joints[0];
	double s1 = sin(local_current_joints[1]);
	double c1 = cos(local_current_joints[1]);
	double s2 = sin(local_current_joints[2]);
	double c2 = cos(local_current_joints[2]);
	double s3 = sin(local_current_joints[3]);
	double c3 = cos(local_current_joints[3]);
	double s4 = sin(local_current_joints[4]);
	double c4 = cos(local_current_joints[4]);
	double s5 = sin(local_current_joints[5]);
	double c5 = cos(local_current_joints[5]);
	double s6 = sin(local_current_joints[6]);
	double c6 = cos(local_current_joints[6]);

	// Proste zadanie kinematyki.
	local_current_end_effector_frame(0,0) = (c1 * s4 * c5 + s1 * s5) * c6 + c1 * c4 * s6; //NX
	local_current_end_effector_frame(0,1) = -(c1 * s4 * c5 + s1 * s5) * s6 + c1 * c4 * c6; //OX
	local_current_end_effector_frame(0,2) = c1 * s4 * s5 - s1 * c5; //AX
	local_current_end_effector_frame(0,3) = c1 * (a2 * c2 + a3 * c3 + d5 * c4); //PX
	local_current_end_effector_frame(1,0) = (s1 * s4 * c5 - c1 * s5) * c6 + s1 * c4 * s6; //NY
	local_current_end_effector_frame(1,1) = -(s1 * s4 * c5 - c1 * s5) * s6 + s1 * c4 * c6; //OY
	local_current_end_effector_frame(1,2) = s1 * s4 * s5 + c1 * c5; //AY
	local_current_end_effector_frame(1,3) = s1 * (a2 * c2 + a3 * c3 + d5 * c4) + d0; //PY
	local_current_end_effector_frame(2,0) = c4 * c5 * c6 - s4 * s6; //NZ
	local_current_end_effector_frame(2,1) = -c4 * c5 * s6 - s4 * c6; //OZ
	local_current_end_effector_frame(2,2) = c4 * s5; //AZ
	local_current_end_effector_frame(2,3) = -a2 * s2 - a3 * s3 - d5 * s4; //PZ

}//:: direct_kinematics_transform()


void model_with_wrist::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{

	// Stale
	const double EPS = 1e-10;

	// Zmienne pomocnicze.
	double Nx, Ox, Ax, Px;
	double Ny, Oy, Ay, Py;
	double Nz, Oz, Az, Pz;
	double s1, c1, s2, c2, s4, c4, s5, c5;
	double E, F, K, ro, G, H;
	double t6, t_ok;

	// Przepisanie zmiennych.
	Nx = local_desired_end_effector_frame(0,0);
	Ny = local_desired_end_effector_frame(1,0);
	Nz = local_desired_end_effector_frame(2,0);
	Ox = local_desired_end_effector_frame(0,1);
	Oy = local_desired_end_effector_frame(1,1);
	Oz = local_desired_end_effector_frame(2,1);
	Ax = local_desired_end_effector_frame(0,2);
	Ay = local_desired_end_effector_frame(1,2);
	Az = local_desired_end_effector_frame(2,2);
	Px = local_desired_end_effector_frame(0,3);
	Py = local_desired_end_effector_frame(1,3) - local_current_joints[0];
	Pz = local_desired_end_effector_frame(2,3);

	//  Wyliczenie Theta1.
	local_desired_joints[1] = (atan2(Py, Px));
	s1 = sin(local_desired_joints[1]);
	c1 = cos(local_desired_joints[1]);

	// Wyliczenie Theta5.
	c5 = Ay * c1 - Ax * s1;
	// Sprawdzenie bledow numerycznych.
	if (fabs(c5 * c5 - 1) > EPS)
		s5 = sqrt(1 - c5 * c5);
	else
		s5 = 0;

	double cj_tmp;
	double dj_translation;
	// Sprawdzenie rozwiazania.
	if (local_current_joints[5] > M_PI) {
		cj_tmp = local_current_joints[5] - 2*M_PI;
		dj_translation = 2*M_PI;
	} else if (local_current_joints[5] < -M_PI) {
		cj_tmp = local_current_joints[5] + 2*M_PI;
		dj_translation = -2*M_PI;
	} else {
		cj_tmp = local_current_joints[5];
		dj_translation = 0.0;
	}

	// Niejednoznacznosc - uzywamy rozwiazanie blizsze poprzedniemu.
	if (cj_tmp > 0)
		local_desired_joints[5] = atan2(s5, c5);
	else
		local_desired_joints[5] = atan2(-s5, c5);

	// Dodanie przesuniecia.
	local_desired_joints[5] += dj_translation;

	// Wyliczenie Theta4 i Theta6.
	if (fabs(s5) < EPS) {
		printf("Osobliwosc\n");
		// W przypadku osobliwosci katowi theta4 przypisywana wartosc poprzednia.
		local_desired_joints[4] = local_current_joints[4];
		t6 = atan2(c1 * Nx + s1 * Ny, c1 * Ox + s1 * Oy);

		// Sprawdzenie warunkow.
		t_ok = t6 + local_desired_joints[4];
		if (fabs(t_ok - local_current_joints[6])
				> fabs(t6 - M_PI + local_desired_joints[4] - (local_current_joints[6])))
			t_ok = t6 - M_PI + local_desired_joints[4];
		if (fabs(t_ok - local_current_joints[6])
				> fabs(t6 + M_PI + local_desired_joints[4] - (local_current_joints[6])))
			t_ok = t6 + M_PI + local_desired_joints[4];

		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 - 2*M_PI + local_desired_joints[4]
				- (local_current_joints[6])))
			t_ok = t6 - 2*M_PI + local_desired_joints[4];
		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 + 2*M_PI + local_desired_joints[4]
				- (local_current_joints[6])))
			t_ok = t6 + 2*M_PI + local_desired_joints[4];

		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 - local_desired_joints[4] - (local_current_joints[6])))
			t_ok = t6 - local_desired_joints[4];
		if (fabs(t_ok - local_current_joints[6])
				> fabs(t6 - M_PI - local_desired_joints[4] - (local_current_joints[6])))
			t_ok = t6 - M_PI - local_desired_joints[4];
		if (fabs(t_ok - local_current_joints[6])
				> fabs(t6 + M_PI - local_desired_joints[4] - (local_current_joints[6])))
			t_ok = t6 + M_PI - local_desired_joints[4];

		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 - 2*M_PI - local_desired_joints[4]
				- (local_current_joints[6])))
			t_ok = t6 - 2*M_PI - local_desired_joints[4];
		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 + 2*M_PI - local_desired_joints[4]
				- (local_current_joints[6])))
			t_ok = t6 + 2*M_PI - local_desired_joints[4];

		local_desired_joints[6] = t_ok;
	} else {
		t6 = atan2(-s1 * Ox + c1 * Oy, s1 * Nx - c1 * Ny);
		t_ok = t6;

		// Sprawdzenie warunkow.
		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 - M_PI - (local_current_joints[6])))
			t_ok = t6 - M_PI;
		if (fabs(t_ok - local_current_joints[6]) > fabs(t6 + M_PI - (local_current_joints[6])))
			t_ok = t6 + M_PI;

		local_desired_joints[6] = t_ok;
		t_ok = atan2(c1 * Ax + s1 * Ay, Az);

		if (fabs(t_ok - local_current_joints[4]) > fabs(t_ok - M_PI - (local_current_joints[4])))
			t_ok = t_ok - M_PI;
		if (fabs(t_ok - local_current_joints[4]) > fabs(t_ok + M_PI - (local_current_joints[4])))
			t_ok = t_ok + M_PI;
		local_desired_joints[4] = t_ok;
	}//: else

	// Wyliczenie Theta2.
	c4 = cos(local_desired_joints[4]);
	s4 = sin(local_desired_joints[4]);

	E = c1 * Px + s1 * Py - c4 * d5;
	F = -Pz - s4 * d5;
	G = 2* E * a2;
	H = 2* F * a2;
	K = E * E + F * F + a2 * a2 - a3 * a3;
	ro = sqrt(G * G + H * H);

	local_desired_joints[2] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro)))) - atan2(G, H);

	// Wyliczenie Theta3.
	s2 = sin(local_desired_joints[2]);
	c2 = cos(local_desired_joints[2]);
	local_desired_joints[3] = atan2(F - a2 * s2, E - a2 * c2);

	// Tor. Nie bierze udzialu w tym rozwiazaniu.
	local_desired_joints[0] = local_current_joints[0];

	// Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
	check_joints(local_desired_joints);

}//: inverse_kinematics_transform()

}// namespace irp6ot
}// namespace kinematic
}// namespace mrrocpp
