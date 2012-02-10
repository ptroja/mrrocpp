/*!
 * @file
 * @brief File containing methods of the model_with_wrist class.
 *
 * The model_with_wrist kinematic model utilizes six (with an additional one in the wrist) IRP-6p DOFs.
 *
 * @author tkornuta
 * @date 24.02.2007
 *
 * @ingroup KINEMATICS IRP6P_KINEMATICS irp6p_m
 */

#include <cstdio>
#include <cmath>

// for MacOS compatibility, where isnan() is implemented as a function in the std:: namespace
// using std::isnan;

#include "base/lib/com_buf.h"
#include "robot/irp6p_m/kinematic_model_irp6p_with_wrist.h"

namespace mrrocpp {
namespace kinematics {
namespace irp6p {

model_with_wrist::model_with_wrist(int _number_of_servos) :
		number_of_servos(_number_of_servos)
{
	// Ustawienie etykiety modelu kinematycznego.
	set_kinematic_model_label("Switching to kinematic model with active wrist");

	// Ustawienie parametrow kinematycznych.
	set_kinematic_parameters();

	// Przeliczac do globalnego ukladu odniesienia.
	global_frame_computations = true;
	// Wykonywac przeliczenia zwiazane z narzedziem.
	attached_tool_computations = true;

}

void model_with_wrist::set_kinematic_parameters(void)
{
	/* -----------------------------------------------------------------------
	 Numery osi:
	 0 - kolumna obrotowa: os FI
	 1 - ramie dolne:     os TETA
	 2 - ramie gorne:    os ALFA
	 3 - pochylnie kisci: os T
	 4 - obrot kisci:    os V
	 5 - obrot kisci:    os N
	 6 - chwytak
	 ------------------------------------------------------------------------- */

	/* -----------------------------------------------------------------------
	 Dlugosci czlonow robota [m].
	 ------------------------------------------------------------------------- */
	a2 = 0.455;
	a3 = 0.67;
	d5 = 0.19;
	d6 = 0.095;
	d7 = 0.20;

	/* -----------------------------------------------------------------------
	 Ustawienie standardowego narzedzia [m].
	 ------------------------------------------------------------------------- */
	// Ustawienie srodka szczek chwytaka.
	tool.set_translation_vector(0, 0, 0.25);

	/* -----------------------------------------------------------------------
	 Ustawienie pozycji ukladu bazowego w globalnym ukladzie odniesienia [m].
	 ------------------------------------------------------------------------- */
	lib::Homog_matrix tmp_global_base_m;
	// Macierz rotacji.
	tmp_global_base_m(0, 0) = 0.999382684525334;
	tmp_global_base_m(1, 0) = -0.032112349151151;
	tmp_global_base_m(2, 0) = 0.014249452723906;
	tmp_global_base_m(0, 1) = 0.032121691496651;
	tmp_global_base_m(1, 1) = 0.999483874031384;
	tmp_global_base_m(2, 1) = -0.000427184516840;
	tmp_global_base_m(0, 2) = -0.014228380312960;
	tmp_global_base_m(1, 2) = 0.000884637222621;
	tmp_global_base_m(2, 2) = 0.999898380141931;
	// Wektor przesuniecia.
	tmp_global_base_m(0, 3) = -0.072793938982293;
	tmp_global_base_m(1, 3) = 2.106338769845522;
	tmp_global_base_m(2, 3) = -0.030202915816185;
	// Ustawienie macierzy.
	global_base = tmp_global_base_m;

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przelozenie przekladni kata obrotu walu silnika napedowego kolumny.
	 *  gear_0    =  H_0 * K_0
	 *  H_0        - przelozenie przekladni harmonicznej kolumny
	 *  K_0        - 1 lub -1 w zaleznosci od tego czy dodatni kierunek ruchu
	 *              walu silnika jest zgodny z dodatnim kierunkiem ruchu osi
	 ------------------------------------------------------------------------- */
	double H_0 = 158;
	double K_0 = -1;
	gear[0] = (H_0 * K_0);
	theta[0] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnikow - przelozenie przekladni kata obrotu walu silnika napedowego ramienia dolnego.
	 *  gear_1    =  (2 * M_PI / S_r) * K_1
	 gdzie
	 * S_r		- Skok sruby kulowej ramion w [mm]
	 * K_1		- kierunek obrotu osi nr 2 wzgledem dodatniego kierunku
	 * theta1_0		- poczatkowa wartosc
	 parametry przekladni srubowo-tocznych dla ramion wyliczone z wzorow
	 * sl123	= l1**2 + l2**2 + l3**2
	 * mi1		= 2*l1*(l2*cl + l3*sl)
	 * ni1		= 2*l1*(l3*cl - l2*sl)
	 ------------------------------------------------------------------------- */
	sl123 = 7.789525e+04;
	mi1 = 6.090255e+04;
	ni1 = -2.934668e+04;

	double S_r = 5;
	double K_1 = 1;
	gear[1] = (2 * M_PI / S_r) * K_1;
	theta[1] = 2.203374e+02; // l02

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przelozenie przekladni kata obrotu walu silnika napedowego ramienia gornego.
	 *  gear_2    =  (2 * M_PI / S_r) * K_2
	 gdzie
	 * S_r 		- Skok sruby kulowej ramion w [mm]
	 * K_2		- kierunek obrotu osi nr 2 wzgledem dodatniego kierunku
	 * theta2_0	- poczatkowa wartosc
	 parametry przekladni srubowo-tocznych dla ramion wyliczone z wzorow
	 * mi2		= -2 * l1 * l2
	 * ni2		= -2 * l1 * l3
	 ------------------------------------------------------------------------- */
	mi2 = -4.410000e+04;
	ni2 = -5.124000e+04;

	double K_2 = 1;
	gear[2] = (2 * M_PI / S_r) * K_2;
	theta[2] = 1.838348e+02; // l03

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przekladni kata obrotu walu silnika napedowego pochylenie kisci T.
	 *  gear_3 = H_3*K_3
	 gdzie
	 *  H_3        przelozenie przekladni harmonicznej osi nr 4
	 *  K_3        = -1 lub 1 w zaleznosci od tego czy dodatnie inkrementy
	 *              odpowiadaja wzrostowi lub maleniu kata t234
	 ------------------------------------------------------------------------- */
	double H_3 = 128;
	double K_3 = -1;
	gear[3] = H_3 * K_3;
	theta[3] = 1.570796e+00; // T2340

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przekladni kata obrotu walu silnika napedowego pochylenie kisci V.
	 * gear_4 = H_4*S_4*K_4;
	 gdzie
	 *  H_4        = przelozenie przekladni harmonicznej osi nr 5
	 *  S_4        = przelozenie przekladni stozkowej
	 *  K_4        = 1 lub -1 zaleznosci od zgodnosci kierunkow wzrostu
	 *				kata t4 i inkrementow
	 ------------------------------------------------------------------------- */
	double H_4 = 128;
	double S_4 = 0.6;
	double K_4 = -1;
	gear[4] = H_4 * S_4 * K_4;
	theta[4] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Obliczanie wspolczynnika - przekladni kata obrotu walu silnika napedowego pochylenie kisci N.
	 * gear_5 = H_5*K_5;
	 gdzie
	 *  H_5        = przelozenie przekladni harmonicznej osi nr 6
	 *  K_5        = 1 lub -1 zaleznosci od zgodnosci kierunkow wzrostu
	 *				kata t5 i inkrementow
	 ------------------------------------------------------------------------- */
	double H_5 = 288.8845;
	double K_5 = 1;
	gear[5] = H_5 * K_5;
	theta[5] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Wspolczynniki uzywane przy obliczeniach zwarcia/rozwarcia szczek:
	 Obliczenia wartosci wspolrzednych wewnetrznych na podstawie odczytow enkoderow silnikow
	 * joint[6] = dir_a_6 * motor[6]^2 + dir_b_6 * motor[6] + dir_c_6
	 uzywane wspolczynniki
	 * dir_a_6, dir_b_6, dir_c_6

	 Obliczenia wartosci polozen silnikow na podstawie wspolrzednych wewnetrznych
	 * motor[6] = inv_a_6 * sqrt(inv_b_6 + inv_c_6 * joint[6]) + inv_d_6
	 uzywane wspolczynniki
	 * inv_a_6, inv_b_6, inv_c_6, inv_d_6
	 ------------------------------------------------------------------------- */
	dir_a_6 = -0.00000000283130;
	dir_b_6 = 0.00001451910074;
	dir_c_6 = 0.074;
	inv_a_6 = 0.3531946456e-5;
	inv_b_6 = 0.2622172716e19;
	inv_c_6 = -0.2831300000e20;
	inv_d_6 = -2564.034320;
	theta[6] = 0.000000e+00;
	gear[6] = 0.000000e+00;

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji - odczyty z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_motor_position[0] = -15.9; // kolumna [rad]
	synchro_motor_position[1] = -5; // ramie d. [rad]
	synchro_motor_position[2] = -8.527; // ramie g. [rad]
	synchro_motor_position[3] = 151.31; // kisc T [rad]
	synchro_motor_position[4] = 435.25; // kisc V [rad]
	synchro_motor_position[5] = 791.0; // kisc N [rad]
	synchro_motor_position[6] = 4830; // chwytak [-]

	/* -----------------------------------------------------------------------
	 Polozenia synchronizacji we wspolrzednych wewnetrznych - obliczone na podstawie z enkoderow silnikow.
	 ------------------------------------------------------------------------- */
	synchro_joint_position[0] = synchro_motor_position[0] - gear[0] * theta[0];
	synchro_joint_position[1] = synchro_motor_position[1] - gear[1] * theta[1];
	synchro_joint_position[2] = synchro_motor_position[2] - gear[2] * theta[2];
	synchro_joint_position[3] = synchro_motor_position[3] - gear[3] * theta[3];
	synchro_joint_position[4] = synchro_motor_position[4] - gear[4] * theta[4] - synchro_motor_position[3];
	synchro_joint_position[5] = synchro_motor_position[5] - gear[5] * theta[5];
	synchro_joint_position[6] = synchro_motor_position[6] - gear[6] * theta[6];

	/* -----------------------------------------------------------------------
	 Zakresy ruchu walow silnikow w radianach.
	 ------------------------------------------------------------------------- */
	lower_limit_axis[0] = -470;
	lower_limit_axis[1] = -110;
	lower_limit_axis[2] = -80;
	lower_limit_axis[3] = -70;
	lower_limit_axis[4] = -80;
	lower_limit_axis[5] = -1000;
	lower_limit_axis[6] = -2000;

	upper_limit_axis[0] = 450;
	upper_limit_axis[1] = 100;
	upper_limit_axis[2] = 100;
	upper_limit_axis[3] = 380;
	upper_limit_axis[4] = 490;
	upper_limit_axis[5] = 3000;
	upper_limit_axis[6] = 5000;

	/* -----------------------------------------------------------------------
	 Zakresy ruchu poszczegolnych stopni swobody (w radianach lub milimetrach).
	 ------------------------------------------------------------------------- */
	lower_limit_joint[0] = -170.0 * M_PI / 180.0;
	lower_limit_joint[1] = -130.0 * M_PI / 180.0;
	lower_limit_joint[2] = -35.0 * M_PI / 180.0;
	lower_limit_joint[3] = -90.0 * M_PI / 180.0;
	lower_limit_joint[4] = -10.0; // -M_PI
	lower_limit_joint[5] = -2.88;
	lower_limit_joint[6] = 0.053;

	upper_limit_joint[0] = 170.0 * M_PI / 180.0; // [rad]
	upper_limit_joint[1] = -50.0 * M_PI / 180.0;
	upper_limit_joint[2] = 40.0 * M_PI / 180.0;
	upper_limit_joint[3] = 92 * M_PI / 180.0;
	upper_limit_joint[4] = 10.0; //M_PI
	upper_limit_joint[5] = 2.93;
	upper_limit_joint[6] = 0.091;

} //: set_kinematic_parameters

void model_with_wrist::check_motor_position(const lib::MotorArray & motor_position) const
{

	if (motor_position[0] < lower_limit_axis[0]) // Kat f1 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_0));
	else if (motor_position[0] > upper_limit_axis[0]) // Kat f1 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_0));

	if (motor_position[1] < lower_limit_axis[1]) // Kat f2 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_1));
	else if (motor_position[1] > upper_limit_axis[1]) // Kat f2 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_1));

	if (motor_position[2] < lower_limit_axis[2]) // Kat f3 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_2));
	else if (motor_position[2] > upper_limit_axis[2]) // Kat f3 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_2));

	if (motor_position[3] < lower_limit_axis[3]) // Kat f4 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_3));
	else if (motor_position[3] > upper_limit_axis[3]) // Kat f4 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_3));

	if (motor_position[4] < lower_limit_axis[4]) // Kat f5 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_4));
	else if (motor_position[4] > upper_limit_axis[4]) // Kat f5 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_4));

	if (motor_position[5] < lower_limit_axis[5]) // Kat f6 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_5));
	else if (motor_position[5] > upper_limit_axis[5]) // Kat f6 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_5));

	if (number_of_servos > 6) {
		if (motor_position[6] < lower_limit_axis[6]) // Kat f7 mniejszy od minimalnego
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_LIMIT_AXIS_6));
		else if (motor_position[6] > upper_limit_axis[6]) // Kat f7 wiekszy od maksymalnego
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_LIMIT_AXIS_6));
	}
} //: check_motor_position

void model_with_wrist::check_joints(const lib::JointArray & q) const
{
	if (isnan(q[0]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA1));

	if (q[0] < lower_limit_joint[0]) // kat q1 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA1_LIMIT));
	else if (q[0] > upper_limit_joint[0]) // kat q1 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA1_LIMIT));

	if (isnan(q[1]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA2));
	if (q[1] < lower_limit_joint[1]) // dlugosc silownika mniejsza od minimalnej
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA2_LIMIT));

	if (q[1] > upper_limit_joint[1]) // dlugosc silownika wieksza od maksymalnej
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA2_LIMIT));

	if (isnan(q[2]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA3));
	if (q[2] < lower_limit_joint[2]) // dlugosc silownika mniejsza od minimalnej
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA3_LIMIT));

	if (q[2] > upper_limit_joint[2]) // dlugosc silownika wieksza od maksymalnej
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA3_LIMIT));

	if (isnan(q[3]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA4));
	if (q[3] < lower_limit_joint[3]) // kat q3 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA4_LIMIT));

	if (q[3] > upper_limit_joint[3])
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA4_LIMIT));

	if (isnan(q[4]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA5));
	if (q[4] < lower_limit_joint[4]) // kat q4 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA5_LIMIT));

	if (q[4] > upper_limit_joint[4]) // kat q4 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA5_LIMIT));

	if (isnan(q[5]))
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA6));
	if (q[5] < lower_limit_joint[5]) // kat q5 mniejszy od minimalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA6_LIMIT));

	if (q[5] > upper_limit_joint[5]) // kat q5 wiekszy od maksymalnego
		BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA6_LIMIT));

	if (number_of_servos > 6) {
		//***szczeki chwytaka***
		if (isnan(q[6]))
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(NOT_A_NUMBER_JOINT_VALUE_THETA7));
		if (q[6] < lower_limit_joint[6]) // 6 st. swobody
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_LOWER_THETA7_LIMIT));

		if (q[6] > upper_limit_joint[6]) // 6 st. swobody
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(BEYOND_UPPER_THETA7_LIMIT));
	}
} //: check_joints

void model_with_wrist::mp2i_transform(const lib::MotorArray & local_current_motor_pos, lib::JointArray & local_current_joints)
{

	// zmienne pomocnicze
	double c, d, l;
	double sinus, cosinus;
	double M1, M2;

	// Sprawdzenie ograniczen na poloenia katowe walow silnikow
	check_motor_position(local_current_motor_pos);

	// Przelicznik polozenia walu silnika napedowego kolumny w radianach
	// na kat obrotu kolumny (wspolrzedna wewnetrzna) w radianach
	local_current_joints[0] = (local_current_motor_pos[0] - synchro_motor_position[0]) / gear[0] + theta[0];

	// Przelicznik polozenia walu silnika napedowego ramienia dolnego w radianach
	// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
	l = (local_current_motor_pos[1] - synchro_motor_position[1]) / gear[1] + theta[1];
	M1 = mi1 * mi1 + ni1 * ni1;
	c = l * l - sl123;
	d = sqrt(M1 - c * c);
	cosinus = (mi1 * c - ni1 * d) / M1;
	sinus = -(ni1 * c + mi1 * d) / M1;
	local_current_joints[1] = atan2(sinus, cosinus);

	// Przelicznik polozenia walu silnika napedowego ramienia gornego w radianach
	// na kat obrotu ramienia (wspolrzedna wewnetrzna) w radianach
	l = (local_current_motor_pos[2] - synchro_motor_position[2]) / gear[2] + theta[2];
	M2 = mi2 * mi2 + ni2 * ni2;
	c = l * l - sl123;
	d = sqrt(M2 - c * c);
	cosinus = (mi2 * c - ni2 * d) / M2;
	sinus = -(ni2 * c + mi2 * d) / M2;
	local_current_joints[2] = atan2(sinus, cosinus);

	// Przelicznik polozenia walu silnika napedowego obrotu kisci T w radianach
	// na kat pochylenia kisci (wspolrzedna wewnetrzna) w radianach
	local_current_joints[3] = (local_current_motor_pos[3] - synchro_motor_position[3]) / gear[3];

	// Przelicznik polozenia walu silnika napedowego obrotu kisci V w radianach
	// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
	local_current_joints[4] = (local_current_motor_pos[4] - synchro_motor_position[4]
			- (local_current_motor_pos[3] - synchro_motor_position[3])) / gear[4] + theta[4];

	// Przelicznik polozenia walu silnika napedowego obrotu kisci N w radianach
	// na kat obrotu kisci (wspolrzedna wewnetrzna) w radianach
	local_current_joints[5] = (local_current_motor_pos[5] - synchro_motor_position[5]) / gear[5] + theta[5];

	/*
	 if (number_of_servos > 6) {
	 // Przelicznik polozenia walu silnika szczek na ich zacisniecie
	 local_current_joints[6] = dir_a_6 * (local_current_motor_pos[6] * local_current_motor_pos[6]) - dir_b_6
	 * local_current_motor_pos[6] + dir_c_6;
	 }
	 */
	// Sprawdzenie obliczonych wartosci wspolrzednych wewnetrznych.
	// poprawka w celu dostosowania do konwencji DH
	local_current_joints[2] -= local_current_joints[1] + M_PI_2;
	local_current_joints[3] -= local_current_joints[2] + local_current_joints[1] + M_PI_2;

	check_joints(local_current_joints);

} //: mp2i_transform

void model_with_wrist::i2mp_transform(lib::MotorArray & local_desired_motor_pos_new, const lib::JointArray & local_desired_joints)
{
	// Niejednoznacznosc polozenia dla 3-tej osi (obrot kisci < 180).
	double joint_3_revolution = M_PI;
	// Niejednoznacznosc polozenia dla 4-tej osi (obrot kisci > 360).
	double axis_4_revolution = 2 * M_PI;

	// Sprawdzenie wartosci wspolrzednych wewnetrznych.
	check_joints(local_desired_joints);

	// poprawka w celu uwzglednienia konwencji DH
	lib::JointArray local_desired_joints_tmp(number_of_servos);
	local_desired_joints_tmp = local_desired_joints;

	local_desired_joints_tmp[2] += local_desired_joints_tmp[1] + M_PI_2;
	local_desired_joints_tmp[3] += local_desired_joints_tmp[2];

	// Obliczanie kata obrotu walu silnika napedowego kolumny
	local_desired_motor_pos_new[0] = gear[0] * local_desired_joints_tmp[0] + synchro_joint_position[0];

	// Obliczanie kata obrotu walu silnika napedowego ramienia dolnego
	local_desired_motor_pos_new[1] = gear[1]
			* sqrt(sl123 + mi1 * cos(local_desired_joints_tmp[1]) + ni1 * sin(-local_desired_joints_tmp[1]))
			+ synchro_joint_position[1];

	// Obliczanie kata obrotu walu silnika napedowego ramienia gornego
	local_desired_motor_pos_new[2] = gear[2]
			* sqrt(sl123 + mi2 * cos(local_desired_joints_tmp[2]) + ni2 * sin(-local_desired_joints_tmp[2]))
			+ synchro_joint_position[2];

	// Obliczanie kata obrotu walu silnika napedowego obotu kisci T
	// jesli jest mniejsze od -pi/2
	double tmp_local_desired_joints_tmp3 = local_desired_joints_tmp[3];
	if (tmp_local_desired_joints_tmp3 < lower_limit_joint[3])
		tmp_local_desired_joints_tmp3 += joint_3_revolution;
	local_desired_motor_pos_new[3] = gear[3] * (tmp_local_desired_joints_tmp3 + theta[3]) + synchro_joint_position[3];

	// Obliczanie kata obrotu walu silnika napedowego obrotu kisci V
	local_desired_motor_pos_new[4] = gear[4] * local_desired_joints_tmp[4] + synchro_joint_position[4]
			+ local_desired_motor_pos_new[3];

	// Ograniczenie na obrot.
	while (local_desired_motor_pos_new[4] < lower_limit_axis[4])
		local_desired_motor_pos_new[4] += axis_4_revolution;
	while (local_desired_motor_pos_new[4] > upper_limit_axis[4])
		local_desired_motor_pos_new[4] -= axis_4_revolution;

	// Obliczanie kata obrotu walu silnika napedowego obrotu kisci N
	local_desired_motor_pos_new[5] = gear[5] * local_desired_joints_tmp[5] + synchro_joint_position[5];

	if (number_of_servos > 6) {
		// Obliczenie kata obrotu walu silnika napedowego chwytaka.
		local_desired_motor_pos_new[6] = inv_a_6 * sqrt(inv_b_6 + inv_c_6 * local_desired_joints_tmp[6]) + inv_d_6;
	}
	// Sprawdzenie obliczonych wartosci.
	check_motor_position(local_desired_motor_pos_new);

} //: i2mp_transform

void model_with_wrist::direct_kinematics_transform(const lib::JointArray & local_current_joints, lib::Homog_matrix& local_current_end_effector_frame)
{

	// Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
	check_joints(local_current_joints);

	// poprawka w celu uwzglednienia konwencji DH
	lib::JointArray local_current_joints_tmp(number_of_servos);
	local_current_joints_tmp = local_current_joints;

	local_current_joints_tmp[2] += local_current_joints_tmp[1] + M_PI_2;
	local_current_joints_tmp[3] += local_current_joints_tmp[2];

	// Parametry pomocnicze - przeliczenie zmiennych.
	const double s1 = sin(local_current_joints_tmp[0]);
	const double c1 = cos(local_current_joints_tmp[0]);
	const double s2 = sin(local_current_joints_tmp[1]);
	const double c2 = cos(local_current_joints_tmp[1]);
	const double s3 = sin(local_current_joints_tmp[2]);
	const double c3 = cos(local_current_joints_tmp[2]);
	const double s4 = sin(local_current_joints_tmp[3]);
	const double c4 = cos(local_current_joints_tmp[3]);
	const double s5 = sin(local_current_joints_tmp[4]);
	const double c5 = cos(local_current_joints_tmp[4]);
	const double s6 = sin(local_current_joints_tmp[5]);
	const double c6 = cos(local_current_joints_tmp[5]);

	// Proste zadanie kinematyki.
	local_current_end_effector_frame(0, 0) = (c1 * s4 * c5 + s1 * s5) * c6 + c1 * c4 * s6; //NX
	local_current_end_effector_frame(0, 1) = -(c1 * s4 * c5 + s1 * s5) * s6 + c1 * c4 * c6; //OX
	local_current_end_effector_frame(0, 2) = c1 * s4 * s5 - s1 * c5; //AX
	local_current_end_effector_frame(0, 3) = c1 * (a2 * c2 + a3 * c3 + d5 * c4); //PX
	local_current_end_effector_frame(1, 0) = (s1 * s4 * c5 - c1 * s5) * c6 + s1 * c4 * s6; //NY
	local_current_end_effector_frame(1, 1) = -(s1 * s4 * c5 - c1 * s5) * s6 + s1 * c4 * c6; //OY
	local_current_end_effector_frame(1, 2) = s1 * s4 * s5 + c1 * c5; //AY
	local_current_end_effector_frame(1, 3) = s1 * (a2 * c2 + a3 * c3 + d5 * c4); //PY
	local_current_end_effector_frame(2, 0) = c4 * c5 * c6 - s4 * s6; //NZ
	local_current_end_effector_frame(2, 1) = -c4 * c5 * s6 - s4 * c6; //OZ
	local_current_end_effector_frame(2, 2) = c4 * s5; //AZ
	local_current_end_effector_frame(2, 3) = -a2 * s2 - a3 * s3 - d5 * s4; //PZ

} //: direct_kinematics_transform()

void model_with_wrist::inverse_kinematics_transform(lib::JointArray & local_desired_joints, const lib::JointArray & local_current_joints, const lib::Homog_matrix& local_desired_end_effector_frame)
{

	// poprawka w celu uwzglednienia konwencji DH
	lib::JointArray local_current_joints_tmp(number_of_servos);
	local_current_joints_tmp = local_current_joints;

	local_current_joints_tmp[2] += local_current_joints_tmp[1] + M_PI_2;
	local_current_joints_tmp[3] += local_current_joints_tmp[2];

	// Stale
	const double EPS = 1e-10;

	// Zmienne pomocnicze.
	double Nx, Ox, Ax, Px;
	double Ny, Oy, Ay, Py;
	double Nz, Oz, Az, Pz;
	double s0, c0, s1, c1, s3, c3, s4, c4;
	double E, F, K, ro, G, H;
	double t5, t_ok;

	// Przepisanie zmiennych.
	Nx = local_desired_end_effector_frame(0, 0);
	Ny = local_desired_end_effector_frame(1, 0);
	Nz = local_desired_end_effector_frame(2, 0);
	Ox = local_desired_end_effector_frame(0, 1);
	Oy = local_desired_end_effector_frame(1, 1);
	Oz = local_desired_end_effector_frame(2, 1);
	Ax = local_desired_end_effector_frame(0, 2);
	Ay = local_desired_end_effector_frame(1, 2);
	Az = local_desired_end_effector_frame(2, 2);
	Px = local_desired_end_effector_frame(0, 3);
	Py = local_desired_end_effector_frame(1, 3);
	Pz = local_desired_end_effector_frame(2, 3);

	//  Wyliczenie Theta1.
	local_desired_joints[0] = (atan2(Py, Px));
	s0 = sin(local_desired_joints[0]);
	c0 = cos(local_desired_joints[0]);

	// Wyliczenie Theta5.
	c4 = Ay * c0 - Ax * s0;
	// Sprawdzenie bledow numerycznych.
	if (fabs(c4 * c4 - 1) > EPS)
		s4 = sqrt(1 - c4 * c4);
	else
		s4 = 0;

	double cj_tmp;
	double dj_translation;
	// Sprawdzenie rozwiazania.
	if (local_current_joints_tmp[4] > M_PI) {
		cj_tmp = local_current_joints_tmp[4] - 2 * M_PI;
		dj_translation = 2 * M_PI;
	} else if (local_current_joints_tmp[4] < -M_PI) {
		cj_tmp = local_current_joints_tmp[4] + 2 * M_PI;
		dj_translation = -2 * M_PI;
	} else {
		cj_tmp = local_current_joints_tmp[4];
		dj_translation = 0.0;
	}

	// Niejednoznacznosc - uzywamy rozwiazanie blizsze poprzedniemu.
	if (cj_tmp > 0)
		local_desired_joints[4] = atan2(s4, c4);
	else
		local_desired_joints[4] = atan2(-s4, c4);

	// Dodanie przesuniecia.
	local_desired_joints[4] += dj_translation;

	// Wyliczenie Theta4 i Theta6.
	if (fabs(s4) < EPS) {
		printf("Osobliwosc\n");
		// W przypadku osobliwosci katowi theta4 przypisywana wartosc poprzednia.
		local_desired_joints[3] = local_current_joints_tmp[3];
		t5 = atan2(c0 * Nx + s0 * Ny, c0 * Ox + s0 * Oy);

		// Sprawdzenie warunkow.
		t_ok = t5 + local_desired_joints[3];
		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 - M_PI + local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 - M_PI + local_desired_joints[3];
		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 + M_PI + local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 + M_PI + local_desired_joints[3];

		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 - 2 * M_PI + local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 - 2 * M_PI + local_desired_joints[3];
		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 + 2 * M_PI + local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 + 2 * M_PI + local_desired_joints[3];

		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 - local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 - local_desired_joints[3];
		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 - M_PI - local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 - M_PI - local_desired_joints[3];
		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 + M_PI - local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 + M_PI - local_desired_joints[3];

		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 - 2 * M_PI - local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 - 2 * M_PI - local_desired_joints[3];
		if (fabs(t_ok - local_current_joints_tmp[5])
				> fabs(t5 + 2 * M_PI - local_desired_joints[3] - (local_current_joints_tmp[5])))
			t_ok = t5 + 2 * M_PI - local_desired_joints[3];

		local_desired_joints[5] = t_ok;
	} else {
		t5 = atan2(-s0 * Ox + c0 * Oy, s0 * Nx - c0 * Ny);
		t_ok = t5;

		// Sprawdzenie warunkow.
		if (fabs(t_ok - local_current_joints_tmp[5]) > fabs(t5 - M_PI - (local_current_joints_tmp[5])))
			t_ok = t5 - M_PI;
		if (fabs(t_ok - local_current_joints_tmp[5]) > fabs(t5 + M_PI - (local_current_joints_tmp[5])))
			t_ok = t5 + M_PI;

		local_desired_joints[5] = t_ok;
		t_ok = atan2(c0 * Ax + s0 * Ay, Az);

		if (fabs(t_ok - local_current_joints_tmp[3]) > fabs(t_ok - M_PI - (local_current_joints_tmp[3])))
			t_ok = t_ok - M_PI;
		if (fabs(t_ok - local_current_joints_tmp[3]) > fabs(t_ok + M_PI - (local_current_joints_tmp[3])))
			t_ok = t_ok + M_PI;
		local_desired_joints[3] = t_ok;
	} //: else

	// Wyliczenie Theta2.
	c3 = cos(local_desired_joints[3]);
	s3 = sin(local_desired_joints[3]);

	E = c0 * Px + s0 * Py - c3 * d5;
	F = -Pz - s3 * d5;
	G = 2 * E * a2;
	H = 2 * F * a2;
	K = E * E + F * F + a2 * a2 - a3 * a3;
	ro = sqrt(G * G + H * H);

	local_desired_joints[1] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro)))) - atan2(G, H);

	// Wyliczenie Theta3.
	s1 = sin(local_desired_joints[1]);
	c1 = cos(local_desired_joints[1]);
	local_desired_joints[2] = atan2(F - a2 * s1, E - a2 * c1);

	// poprawka w celu dostosowania do konwencji DH
	local_desired_joints[2] -= local_desired_joints[1] + M_PI_2;
	local_desired_joints[3] -= local_desired_joints[2] + local_desired_joints[1] + M_PI_2;

	// Sprawdzenie ograniczen na wspolrzedne wewnetrzne.
	check_joints(local_desired_joints);

} //: inverse_kinematics_transform()

} // namespace irp6p
} // namespace kinematic
} // namespace mrrocpp
