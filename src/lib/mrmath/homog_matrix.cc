// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector
//				- deklaracja metod klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------

#include <math.h>
#include <stdio.h>
#include <iostream>

#include "lib/mis_fun.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

// ******************************************************************************************
//                                     definicje skladowych klasy Homog_matrix
// ******************************************************************************************

Homog_matrix::Homog_matrix()
{
	// Tworzy macierz jednostkowa
	// 			| 1 0 0 0 |
	// 			| 0 1 0 0 |
	// 			| 0 0 1 0 |

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++) {
			if (i == j)
				matrix_m[j][i] = 1;
			else
				matrix_m[j][i] = 0;
		}
}

Homog_matrix::Homog_matrix(K_vector versor_x, K_vector versor_y, K_vector versor_z, K_vector angles)
{
	matrix_m[0][0] = 1;
	matrix_m[1][0] = versor_x[2] * angles[0] + versor_y[2] * angles[1] + versor_z[2] * angles[2];
	matrix_m[2][0] = -1* ( versor_x[1]*angles[0]+versor_y[1]*angles[1]+versor_z[1]*angles[2]);

	matrix_m[0][1] = -1*(versor_x[2]*angles[0]+versor_y[2]*angles[1]+versor_z[2]*angles[2]);
	matrix_m[1][1] = 1;
	matrix_m[2][1] = versor_x[0]*angles[0]+versor_y[0]*angles[1]+versor_z[0]*angles[2];

	matrix_m[0][2] = versor_x[1]*angles[0]+versor_y[1]*angles[1]+versor_z[1]*angles[2];
	matrix_m[1][2] = -1*(versor_x[0]*angles[0]+versor_y[0]*angles[1]+versor_z[0]*angles[2]);
	matrix_m[2][2] = 1;

	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;

}

Homog_matrix::Homog_matrix(const Xyz_Euler_Zyz_vector l_vector)
{
	set_from_xyz_euler_zyz (l_vector);
}

Homog_matrix::Homog_matrix(const Xyz_Rpy_vector l_vector) {

	set_from_xyz_rpy (l_vector);
}

Homog_matrix::Homog_matrix(const Xyz_Angle_Axis_vector l_vector)
{
	set_from_xyz_angle_axis (l_vector);
}

Homog_matrix::Homog_matrix(K_vector angles)
{
	matrix_m[0][0] = 1;
	matrix_m[0][0] = angles[2];
	matrix_m[0][0] = -angles[1];

	matrix_m[0][1] = -angles[2];
	matrix_m[1][1] = 1;
	matrix_m[2][1] = angles[0];

	matrix_m[0][2] = angles[1];
	matrix_m[1][2] = -angles[0];
	matrix_m[2][2] = 1;

	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;

}

Homog_matrix::Homog_matrix(double r[3][3], double t[3])
{
	// utworznie macierzy jednorodnej na podstawie
			// - macierzy rotacji r
			// - wektora przesuniecia t

			// i - i-ta kolumna
			// j - j-ty wiersz

			// uzupelnianie macierzy przeksztalcenia wierszami
			for(int j=0; j<3; j++)
			{
				for(int i=0; i<3; i++)
				matrix_m[i][j] = r[i][j];

				matrix_m[j][3] = t[j];
			}//: for
		}

		// Utworzenie macierzy jednorodnej na podstawie zawartosci tablicy podanej jako argument.
		Homog_matrix::Homog_matrix(const frame_tab frame)
		{
			set_from_frame_tab(frame);
		}//: Homog_matrix::Homog_matrix(frame_tab frame)


		// kontruktor kopiujacy
		// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
		Homog_matrix::Homog_matrix(const Homog_matrix &wzor)
		{
			set_from_frame_tab(wzor.matrix_m);
		}//: Homog_matrix::Homog_matrix(const Homog_matrix &wzor)

		Homog_matrix::Homog_matrix(double x, double y, double z)
		{
			// Tworzy macierz jednorodna
			// 			| 1 0 0 x |
			// 			| 0 1 0 y |
			// 			| 0 0 1 z |

			remove_rotation();

			matrix_m[0][3] = x;
			matrix_m[1][3] = y;
			matrix_m[2][3] = z;

		}//: Homog_matrix::Homog_matrix(double x, double y, double z)


		// Utworzenie macierzy jednorodnej na podstawie jej 12 elementow (notacja z Craiga)
		Homog_matrix::Homog_matrix (double r11, double r12, double r13, double t1, double r21, double r22, double r23,double t2, double r31, double r32,
	 double r33, double t3)
{
	matrix_m[0][0] = r11; 	matrix_m[0][1] = r12; 	matrix_m[0][2] = r13; 	matrix_m[0][3] = t1;
	matrix_m[1][0] = r21; 	matrix_m[1][1] = r22; 	matrix_m[1][2] = r23;	matrix_m[1][3] = t2;
	matrix_m[2][0] = r31; 	matrix_m[2][1] = r32; 	matrix_m[2][2] = r33; 	matrix_m[2][3] = t3;
}


// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
Homog_matrix::Homog_matrix(double kx, double ky, double kz, double alfa, double x, double y, double z)
{
	// Wywolanie metody, ktora odpowiednio wypelni macierz matrix.
	set_from_xyz_angle_axis(kx, ky, kz, alfa, x, y, z);
}//: Homog_matrix::Homog_matrix(double kx, double ky, double kz, double alfa, double x, double y, double z)


// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
Homog_matrix::Homog_matrix (POSE_SPECIFICATION mtr_ps, const K_vector axis_with_angle, const K_vector translation)
{
	switch (mtr_ps)
	{
		case MTR_XYZ_ANGLE_AXIS:
			set_from_xyz_angle_axis (axis_with_angle, translation);
		break;
		default:
		break;
	}

} //: Homog_matrix();


Homog_matrix::Homog_matrix(POSE_SPECIFICATION mtr_ps, const Ft_v_vector translation_and_axis_with_angle)
{
	switch (mtr_ps)
	{
		case MTR_XYZ_ANGLE_AXIS:
			set_from_xyz_angle_axis (translation_and_axis_with_angle);
		break;
		default:
		break;
	}
}

Homog_matrix::Homog_matrix(POSE_SPECIFICATION mtr_ps, const double t[6])
{
	switch (mtr_ps)
	{
		case MTR_XYZ_ANGLE_AXIS:
			set_from_xyz_angle_axis (t);
			break;
		default:
			break;
	}
}


// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie.
Homog_matrix::Homog_matrix(POSE_SPECIFICATION mtr_ps, double x, double y, double z, double alfa, double beta, double gamma)
{
	// Wywolanie metody, ktora odpowiednio wypelni macierz matrix.
	switch (mtr_ps)
	{
			case MTR_XYZ_ANGLE_AXIS:
			set_from_xyz_angle_axis (alfa, beta, gamma, x, y, z);
			break;
		default:
			break;
	}

} //: Homog_matrix();


// Zwrocenie obecnej tablicy, zawierajacej dane macierzy jednorodnej.
void Homog_matrix::get_frame_tab(frame_tab frame) const
{
	copy_frame_tab(frame, matrix_m);
}//: get_frame_tab


// Ustawienie tablicy, ktora zawiera dane macierzy jednorodnej.
void Homog_matrix::set_from_frame_tab(const frame_tab frame)
{
	copy_frame_tab(matrix_m, frame);
}//: set_from_frame_tab



// Przeksztalcenie do formy XYZ_EULER_ZYZ i zwrocenie w tablicy.
/*
void Homog_matrix::get_xyz_euler_zyz(double t[6]) const
{
	double alfa, beta, gamma; // Katy Euler'a Z-Y-Z
	//const double EPS=1.0E-10;
	const double EPS=0.1;

	// Sprawdzenie czy cos(beta) == 1.
	if (matrix[2][2] < 1+EPS && matrix[2][2] > 1-EPS)
	{
		// Osie pierwszego oraz trzeciego obrotu pokrywaja sie.
		// Obrot o kat (alfa + gamma).
		beta = 0.0;
		// Przyjecie, ze jeden jest zero, a drugi zostaje obliczony
		gamma = 0.0;
		alfa = atan2(-matrix[1][0],matrix[0][0]);
	}
	// Sprawdzenie czy cos(beta) == -1.
	else if(matrix[2][2] < -1+EPS && matrix[2][2] > -1-EPS)
	{
		// Osie pierwszego oraz trzeciego obrotu pokrywaja sie, lecz sa skierowanie przeciwnie.
		// Obrot o kat  (alfa - gamma)
		beta = M_PI;
		// Przyjecie, ze jeden jest zero, a drugi zostaje obliczony.
		gamma = 0.0;
		alfa = atan2(-matrix[0][1], matrix[1][1]);
	}
	else
	{
		// Normalne rozwiazanie.
		double sb = sqrt(matrix[0][2]*matrix[0][2] + matrix[1][2]*matrix[1][2]);
		beta = atan2(sb, matrix[2][2]);
//		beta = acos(matrix[2][2]);

		alfa = atan2(matrix[2][1], matrix[2][0]);
		gamma = atan2(matrix[1][2], -matrix[0][2]);

		// Sinus beta.
		// Jesli alfa !=0 oraz |alfa| != 1 (alfa != pi && alfa != -pi)
//		if (alfa > EPS && alfa < -EPS && fabs(alfa) > M_PI +EPS && fabs(alfa) < M_PI - EPS)
//		{
//printf(" (alfa > EPS && alfa < -EPS && fabs(alfa) > M_PI +EPS && fabs(alfa) < M_PI - EPS)\n");
//			sb = matrix[2][1] / sin(alfa);
//		}
//		else
//		{
//printf(" else\n");
//			sb = matrix[2][0] / cos(alfa);
//		}
	};//: else

	// Zredukowanie katow.
	alfa = reduce(alfa, -M_PI, M_PI, 2*M_PI);
	beta = reduce(beta, 0, M_PI, M_PI);
	gamma = reduce(gamma, -M_PI, M_PI, 2*M_PI);

	// Przepisanie wyniku do tablicy
	for(int i = 0;i<3;i++)
		t[i] = matrix[3][i];
	t[3] = alfa;
	t[4] = beta;
	t[5] = gamma;
};//: get_xyz_euler_zyz
*/
void Homog_matrix::get_xyz_euler_zyz(Xyz_Euler_Zyz_vector& l_vector) const
{
	double alfa, beta, gamma; // Katy Euler'a Z-Y-Z
	//const double EPS=1.0E-10;
	const double EPS=0.000001;
	// Sprawdzenie czy cos(beta) == 1.
	if (matrix_m[2][2] < 1+EPS && matrix_m[2][2] > 1-EPS)
	{
		// Osie pierwszego oraz trzeciego obrotu pokrywaja sie.
		// Obrot o kat (alfa + gamma).
		beta = 0.0;
		// Przyjecie, ze jeden jest zero, a drugi zostaje obliczony
		gamma = 0.0;
		alfa = atan2(-matrix_m[0][1],matrix_m[0][0]);
	}
	// Sprawdzenie czy cos(beta) == -1.
	else if(matrix_m[2][2] < -1+EPS && matrix_m[2][2] > -1-EPS)
	{
		// Osie pierwszego oraz trzeciego obrotu pokrywaja sie, lecz sa skierowanie przeciwnie.
		// Obrot o kat  (alfa - gamma)
		beta = M_PI;
		// Przyjecie, ze jeden jest zero, a drugi zostaje obliczony.
		gamma = 0.0;
		alfa = atan2(-matrix_m[1][0], matrix_m[1][1]);
	}
	else
	{
		// Normalne rozwiazanie.
		double sb = sqrt(matrix_m[2][0]*matrix_m[2][0] + matrix_m[2][1]*matrix_m[2][1]);
		beta = atan2(sb, matrix_m[2][2]);
//		beta = acos(matrix[2][2]);

		alfa = atan2(matrix_m[1][2], matrix_m[0][2]);
		gamma = atan2(matrix_m[2][1], -matrix_m[2][0]);

		// Sinus beta.
	}

	// Przepisanie wyniku do tablicy
	for(int i = 0;i<3;i++)
	{
		l_vector[i] = matrix_m[i][3];
	}
	l_vector[3] = alfa;
	l_vector[4] = beta;
	l_vector[5] = gamma;
}//: get_xyz_euler_zyz

// Przeksztalcenie do formy XYZ_EULER_ZYZ dla robota IRP_6 MECHATRONIKAi zwrocenie w tablicy.
void Homog_matrix::get_mech_xyz_euler_zyz(Xyz_Euler_Zyz_vector& l_vector) const
{

	// Przepisanie wyniku do tablicy
	for(int i = 0;i<3;i++)
		l_vector[i] = matrix_m[i][3];
	l_vector[3] = matrix_m[0][0];
	l_vector[4] = matrix_m[0][1];
	l_vector[5] = matrix_m[0][2];
}//: get_mech_xyz_euler_zyz

void Homog_matrix::set_from_xyz_euler_zyz(const Xyz_Euler_Zyz_vector l_vector)
{
	// alfa, beta, gamma - Katy Euler'a Z-Y-Z
	// Zredukowanie katow.
	double alfa = reduce(l_vector[3], -M_PI, M_PI, 2*M_PI);
	double beta = reduce(l_vector[4], 0, M_PI, M_PI);
	double gamma = reduce(l_vector[5], -M_PI, M_PI, 2*M_PI);

	// Obliczenie sinusow/cosinusow.
	const double c_alfa = cos(alfa);
	const double s_alfa = sin(alfa);
	const double c_beta = cos(beta);
	const double s_beta = sin(beta);
	const double c_gamma= cos(gamma);
	const double s_gamma= sin(gamma);

	// Obliczenie macierzy rotacji.
	matrix_m[0][0] = c_alfa*c_beta*c_gamma-s_alfa*s_gamma;
	matrix_m[1][0] = s_alfa*c_beta*c_gamma+c_alfa*s_gamma;
	matrix_m[2][0] = -s_beta*c_gamma;

	matrix_m[0][1] = -c_alfa*c_beta*s_gamma-s_alfa*c_gamma;
	matrix_m[1][1] = -s_alfa*c_beta*s_gamma+c_alfa*c_gamma;
	matrix_m[2][1] = s_beta*s_gamma;

	matrix_m[0][2] = c_alfa*s_beta;
	matrix_m[1][2] = s_alfa*s_beta;
	matrix_m[2][2] = c_beta;

	// Przepisanie polozenia.
	matrix_m[0][3] = l_vector[0];
	matrix_m[1][3] = l_vector[1];
	matrix_m[2][3] = l_vector[2];

}//: set_from_xyz_euler_zyz

// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_EULER_ZYZ dla robota IRP-6_MECHATRONIKA.
void Homog_matrix::set_from_mech_xyz_euler_zyz(const Xyz_Euler_Zyz_vector l_vector)
{
	// alfa, beta, gamma - Katy Euler'a Z-Y-Z
	// Zredukowanie katow.
	double alfa = reduce(l_vector[3], -M_PI, M_PI, 2*M_PI);
	double beta = reduce(l_vector[4], 0, M_PI, M_PI);
	double gamma = reduce(l_vector[5], -M_PI, M_PI, 2*M_PI);

	// Obliczenie sinusow/cosinusow.
	const double s_alfa = sin(alfa);
	const double s_beta = sin(beta);
	const double s_gamma= sin(gamma);

	// Obliczenie macierzy rotacji.
	matrix_m[0][0] = s_alfa;
	matrix_m[0][1] = s_beta;
	matrix_m[0][2] = s_gamma;

	matrix_m[1][0] = 1;
	matrix_m[1][1] = 1;
	matrix_m[1][2] = 1;

	matrix_m[2][0] = 1;
	matrix_m[2][1] = 1;
	matrix_m[2][2] = 1;

	// Przepisanie polozenia.
	matrix_m[0][3] = l_vector[0];
	matrix_m[1][3] = l_vector[1];
	matrix_m[2][3] = l_vector[2];
}//: set_from_mech_xyz_euler_zyz


// notacja i wzory z Craiga - wydanie angielskie str. 41

// UWAGA ponizsze dwie funckje nie byly testowane - po pozytywnych  testach usunac komentarz
// Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
void Homog_matrix::get_xyz_rpy(Xyz_Rpy_vector& l_vector) const
{
	// x, y, z
	l_vector[0] = matrix_m[0][3];
	l_vector[1] = matrix_m[1][3];
	l_vector[2] = matrix_m[2][3];

	// alfa (wokol z) , beta (wokol y), gamma (wokol x)
	l_vector[3] = atan2 (matrix_m[2][1], matrix_m[2][2]);
	l_vector[4] = atan2 (matrix_m[2][0], sqrt (matrix_m[0][0]*matrix_m[0][0] + matrix_m[1][0]*matrix_m[1][0]));
	l_vector[5] = atan2 (matrix_m[1][0], matrix_m[0][0]);
}


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
void Homog_matrix::set_from_xyz_rpy(const Xyz_Rpy_vector l_vector)
{
	// alfa (wokol z) , beta (wokol y), gamma (wokol x)
	const double c_alfa = cos(l_vector[4]);
	const double s_alfa = sin(l_vector[4]);
	const double c_beta = cos(l_vector[5]);
	const double s_beta = sin(l_vector[5]);
	const double c_gamma = cos(l_vector[6]);
	const double s_gamma = sin(l_vector[6]);

	// Obliczenie macierzy rotacji.
	matrix_m[0][0] = c_alfa*c_beta;
	matrix_m[1][0] = s_alfa*c_beta;
	matrix_m[2][0] = -s_beta;

	matrix_m[0][1] = c_alfa*s_beta*s_gamma - s_alfa*c_gamma;
	matrix_m[1][1] = s_alfa*s_beta*s_gamma + c_alfa*c_gamma;
	matrix_m[2][1] = c_beta*s_gamma;

	matrix_m[0][2] = c_alfa*s_beta*c_gamma + s_alfa*s_gamma;
	matrix_m[1][2] = s_alfa*s_beta*c_gamma - c_alfa*s_gamma;
	matrix_m[2][2] = c_beta*c_gamma;

	// Przepisanie polozenia.
	matrix_m[0][3] = l_vector[0];
	matrix_m[1][3] = l_vector[1];
	matrix_m[2][3] = l_vector[2];
}



// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_from_xyz_angle_axis(double kx, double ky, double kz, double alfa, double x, double y, double z)
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	// c_alfa - kosinus kata alfa
	// s_alfa - sinus kata alfa
	// v_alfa = 1 - c_alfa;

	// wartosci poszczegolnych funkcji trygonometrycznych dla kata obrotu
	const double c_alfa = cos(alfa);
	const double s_alfa = sin(alfa);
	const double v_alfa = 1 - c_alfa;

	// macierz rotacji na podstawie wzoru 2.80 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	matrix_m[0][0] = kx*kx*v_alfa + c_alfa;
	matrix_m[1][0] = kx*ky*v_alfa + kz*s_alfa;
	matrix_m[2][0] = kx*kz*v_alfa - ky*s_alfa;

	matrix_m[0][1] = kx*ky*v_alfa - kz*s_alfa;
	matrix_m[1][1] = ky*ky*v_alfa + c_alfa;
	matrix_m[2][1] = ky*kz*v_alfa + kx*s_alfa;

	matrix_m[0][2] = kx*kz*v_alfa + ky*s_alfa;
	matrix_m[1][2] = ky*kz*v_alfa - kx*s_alfa;
	matrix_m[2][2] = kz*kz*v_alfa + c_alfa;

	// uzupelnienie macierzy
	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;

}//: set_from_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS - kat wliczony w dlugos osi.
void Homog_matrix::set_from_xyz_angle_axis(double kx, double ky, double kz, double x, double y, double z)
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okreslona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	const double alfa = sqrt(kx*kx + ky*ky + kz*kz);

	double kx_l, ky_l, kz_l;

	if (alfa > ALFA_SENSITIVITY)
	{
		kx_l = kx / alfa;
		ky_l = ky / alfa;
		kz_l = kz / alfa;
	}
	else
	{
		kx_l = ky_l =  kz_l = 0.0;
	}

	set_from_xyz_angle_axis (kx_l, ky_l, kz_l, alfa, x, y, z);

}//: set_from_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_from_xyz_angle_axis(const K_vector axis_with_angle, const K_vector translation)
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	set_from_xyz_angle_axis (axis_with_angle[0], axis_with_angle[1], axis_with_angle[2], translation[0], translation[1], translation[2]);

}//: set_from_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_from_xyz_angle_axis(const Ft_v_vector translation_and_axis_with_angle)  // kat wliczony w os
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	set_from_xyz_angle_axis (translation_and_axis_with_angle[3], translation_and_axis_with_angle[4], translation_and_axis_with_angle[5],
		 translation_and_axis_with_angle[0], translation_and_axis_with_angle[1], translation_and_axis_with_angle[2]);

}//: set_from_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_from_xyz_angle_axis(const double t[6])  // kat wliczony w os
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	set_from_xyz_angle_axis (t[3], t[4], t[5], t[0], t[1], t[2]);

}//: set_from_xyz_angle_axis

//Wypelnienie macierzy na podstawie parametrow  wejsciowych w postaci kwaternionu
void Homog_matrix::set_from_xyz_quaternion(double eta, double eps1, double eps2, double eps3, double x, double y, double z)
{
	// Macierz rotacji


	matrix_m[0][0] = 2*(eta*eta + eps1*eps1) -1;
	matrix_m[1][0] = 2*(eps1*eps2 + eta*eps3);
	matrix_m[2][0] = 2*(eps1*eps3 - eta*eps2);

	matrix_m[0][1] = 2*(eps1*eps2 - eta*eps3);
	matrix_m[1][1] = 2*(eta*eta + eps2*eps2) - 1;
	matrix_m[2][1] = 2*(eps2*eps3 + eta*eps1);

	matrix_m[0][2] = 2*(eps1*eps3 + eta*eps2);
	matrix_m[1][2] = 2*(eps2*eps3 - eta*eps1);
	matrix_m[2][2] = 2*(eta*eta + eps3*eps3) - 1;

	// Uzupelnienie macierzy wspolrzednymi polozenia

	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;

}// Homog_matrix::set_from_xyz_quaternion(double eta, K_vector eps, double x, double y, double z)


void Homog_matrix::set_from_xyz_angle_axis(const Xyz_Angle_Axis_vector l_vector)  // kat wliczony w os
{
	const double alfa = sqrt(l_vector[3]*l_vector[3] + l_vector[4]*l_vector[4] + l_vector[5]*l_vector[5]);

	double kx, ky, kz;

	if (alfa > ALFA_SENSITIVITY)
	{
		kx = l_vector[3] / alfa;
		ky = l_vector[4] / alfa;
		kz = l_vector[5] / alfa;
	}
	else
	{
		kx = ky =  kz = 0.0;
	}


	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	// c_alfa - kosinus kata alfa
	// s_alfa - sinus kata alfa
	// v_alfa = 1 - c_alfa;

	// wartosci poszczegolnych funkcji trygonometrycznych dla kata obrotu
	const double c_alfa = cos(alfa);
	const double s_alfa = sin(alfa);
	const double v_alfa = 1 - c_alfa;

	// macierz rotacji na podstawie wzoru 2.80 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	matrix_m[0][0] = kx*kx*v_alfa + c_alfa;
	matrix_m[1][0] = kx*ky*v_alfa + kz*s_alfa;
	matrix_m[2][0] = kx*kz*v_alfa - ky*s_alfa;

	matrix_m[0][1] = kx*ky*v_alfa - kz*s_alfa;
	matrix_m[1][1] = ky*ky*v_alfa + c_alfa;
	matrix_m[2][1] = ky*kz*v_alfa + kx*s_alfa;

	matrix_m[0][2] = kx*kz*v_alfa + ky*s_alfa;
	matrix_m[1][2] = ky*kz*v_alfa - kx*s_alfa;
	matrix_m[2][2] = kz*kz*v_alfa + c_alfa;

	// uzupelnienie macierzy
	matrix_m[0][3] = l_vector[0];
	matrix_m[1][3] = l_vector[1];
	matrix_m[2][3] = l_vector[2];
}

void Homog_matrix::get_xyz_angle_axis(K_vector& axis_with_angle, K_vector& translation) const
{

	double t[6];

	get_xyz_angle_axis (t);

	for(int i=0;i<3;i++)
	{
		axis_with_angle[i] = t[3+ i];
		translation[i] = t[i];
	}//: for

}

void Homog_matrix::get_xyz_angle_axis(Ft_v_vector& translation_and_axis_with_angle) const
{

	double t[6];

	get_xyz_angle_axis (t);

	for(int i=0;i<6;i++)
	{
		translation_and_axis_with_angle[i] = t[i];
	}//: for

}

void Homog_matrix::get_xyz_angle_axis(Xyz_Angle_Axis_vector& l_vector) const
{
	// przeksztalcenie macierzy jednorodnej do rozkazu w formie XYZ_ANGLE_AXIS
	const double EPS = zero_eps;
	const double delta = delta_m;

	double Kd[3];	// Kd - K z "daszkiem" - wersor kierunkowy

	// obliczenia zgodne ze wzorami 2.81 i 2.82 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	double value = (matrix_m[0][0]+matrix_m[1][1]+	matrix_m[2][2]-1)/2;

	// wyeliminowanie niedokladnosci obliczeniowej lub bledu programisty
	if(value < -1)
		value = -1;
	else if (value > 1)
		value = 1;

	// kat obrotu
	double alfa = acos(value);

	if((alfa  < M_PI + delta) && (alfa > M_PI - delta))							// kat obrotu 180 stopni = Pi radianow
	{

		Kd[0] = sqrt((matrix_m[0][0]+1)/(double)2);
		Kd[1] = sqrt((matrix_m[1][1]+1)/(double)2);
		Kd[2] = sqrt((matrix_m[2][2]+1)/(double)2);

		// ustalenie znakow paramertow wersora

		if(((Kd[0] < -EPS)||(Kd[0] > EPS)) && ((Kd[1] < -EPS)||(Kd[1] > EPS))
			&& ((Kd[2] < -EPS)||(Kd[2] > EPS)))
		{
			if((matrix_m[0][1] < 0) && (matrix_m[0][2] < 0))
			{
				Kd[1] = Kd[1]*(-1);
				Kd[2] = Kd[2]*(-1);
			}
			else if((matrix_m[0][1] < 0) && (matrix_m[0][2] > 0))
				Kd[1] = Kd[1]*(-1);
			else if((matrix_m[0][1] > 0) && (matrix_m[0][2] < 0))
				Kd[2] = Kd[2]*(-1);
		}
		else if (((Kd[0] > -EPS) && (Kd[0] < EPS)) && ((Kd[1] < -EPS)||(Kd[1] > EPS))
			&& ((Kd[2] < -EPS)||(Kd[2] > EPS)))		// kx==0, ky!=0, kz!=0
		{
			if(matrix_m[1][2] < 0)
				Kd[2] = Kd[2]*(-1);
		}
		else if (((Kd[0] < -EPS) || (Kd[0] > EPS)) && ((Kd[1] > -EPS)&&(Kd[1] < EPS))
			&& ((Kd[2] < -EPS)||(Kd[2] > EPS)))		// kx!=0, ky==0, kz!=0
		{
			if(matrix_m[0][2] < 0)
				Kd[2] = Kd[2]*(-1);
		}
		else if(((Kd[0] < -EPS)||(Kd[0] > EPS)) && ((Kd[1] < -EPS)||(Kd[1] > EPS))
			&& ((Kd[2] > -EPS) && (Kd[2] < EPS)))	// kx!=0 ky!=0 kz==0
		{
			if(matrix_m[0][1] < 0)
				Kd[1] = Kd[1]*(-1);
		}

	}// end kat obrotu 180 stopni
	else if ((alfa < ALFA_SENSITIVITY) && (alfa > -ALFA_SENSITIVITY))									// kat obrotu 0 stopni
	{

		for(int i=0; i<3; i++)
			Kd[i] = 0;

		alfa = 0;

	}
	else																				// standardowe obliczenia
	{
		// sinus kata obrotu alfa
		const double s_alfa = sin(alfa);

		Kd[0] = (1/(2*s_alfa))*(matrix_m[2][1] - matrix_m[1][2]);
		Kd[1] = (1/(2*s_alfa))*(matrix_m[0][2] - matrix_m[2][0]);
		Kd[2] = (1/(2*s_alfa))*(matrix_m[1][0] - matrix_m[0][1]);
	}

	// Przepisanie wyniku do tablicy
	for(int i=0;i<3;i++)
	{
		l_vector[i] = matrix_m[i][3];
		l_vector[3+i] = Kd[i]*alfa;
	}//: for
}


// Przeksztalcenie do formy XYZ_ANGLE_AXIS i zwrocenie w tablicy.
void Homog_matrix::get_xyz_angle_axis(double t[6]) const
{
	// przeksztalcenie macierzy jednorodnej do rozkazu w formie XYZ_ANGLE_AXIS
	const double EPS = zero_eps;
	const double delta = delta_m;

	double Kd[3];	// Kd - K z "daszkiem" - wersor kierunkowy

	// obliczenia zgodne ze wzorami 2.81 i 2.82 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	double value = (matrix_m[0][0]+matrix_m[1][1]+	matrix_m[2][2]-1)/2;

	// wyeliminowanie niedokladnosci obliczeniowej lub bledu programisty
	if(value < -1)
		value = -1;
	else if (value > 1)
		value = 1;

	// kat obrotu
	double alfa = acos(value);

	if((alfa  < M_PI + delta) && (alfa > M_PI - delta))							// kat obrotu 180 stopni = Pi radianow
	{

		Kd[0] = sqrt((matrix_m[0][0]+1)/(double)2);
		Kd[1] = sqrt((matrix_m[1][1]+1)/(double)2);
		Kd[2] = sqrt((matrix_m[2][2]+1)/(double)2);

		// ustalenie znakow paramertow wersora

		if(((Kd[0] < -EPS)||(Kd[0] > EPS)) && ((Kd[1] < -EPS)||(Kd[1] > EPS))
			&& ((Kd[2] < -EPS)||(Kd[2] > EPS)))
		{
			if((matrix_m[0][1] < 0) && (matrix_m[0][2] < 0))
			{
				Kd[1] = Kd[1]*(-1);
				Kd[2] = Kd[2]*(-1);
			}
			else if((matrix_m[0][1] < 0) && (matrix_m[0][2] > 0))
				Kd[1] = Kd[1]*(-1);
			else if((matrix_m[0][1] > 0) && (matrix_m[0][2] < 0))
				Kd[2] = Kd[2]*(-1);
		}
		else if (((Kd[0] > -EPS) && (Kd[0] < EPS)) && ((Kd[1] < -EPS)||(Kd[1] > EPS))
			&& ((Kd[2] < -EPS)||(Kd[2] > EPS)))		// kx==0, ky!=0, kz!=0
		{
			if(matrix_m[1][2] < 0)
				Kd[2] = Kd[2]*(-1);
		}
		else if (((Kd[0] < -EPS) || (Kd[0] > EPS)) && ((Kd[1] > -EPS)&&(Kd[1] < EPS))
			&& ((Kd[2] < -EPS)||(Kd[2] > EPS)))		// kx!=0, ky==0, kz!=0
		{
			if(matrix_m[0][2] < 0)
				Kd[2] = Kd[2]*(-1);
		}
		else if(((Kd[0] < -EPS)||(Kd[0] > EPS)) && ((Kd[1] < -EPS)||(Kd[1] > EPS))
			&& ((Kd[2] > -EPS) && (Kd[2] < EPS)))	// kx!=0 ky!=0 kz==0
		{
			if(matrix_m[0][1] < 0)
				Kd[1] = Kd[1]*(-1);
		}

	}// end kat obrotu 180 stopni
	else if ((alfa < ALFA_SENSITIVITY) && (alfa > -ALFA_SENSITIVITY))									// kat obrotu 0 stopni
	{

		for(int i=0; i<3; i++)
			Kd[i] = 0;

		alfa = 0;

	}
	else																				// standardowe obliczenia
	{
		// sinus kata obrotu alfa
		const double s_alfa = sin(alfa);

		Kd[0] = (1/(2*s_alfa))*(matrix_m[2][1] - matrix_m[1][2]);
		Kd[1] = (1/(2*s_alfa))*(matrix_m[0][2] - matrix_m[2][0]);
		Kd[2] = (1/(2*s_alfa))*(matrix_m[1][0] - matrix_m[0][1]);
	}

	// Przepisanie wyniku do tablicy
	for(int i=0;i<3;i++)
	{
		t[i] = matrix_m[i][3];
		t[3+i] = Kd[i]*alfa;
	}//: for
}//: get_xyz_angle_axis

// Obliczenie kwaternionu na podstawie macierzy rotacji
void Homog_matrix::get_xyz_quaternion(double t[7]) const
{
	double eta, eps1, eps2, eps3;
	double tr; // slad macierzy
	const double EPSILON=1.0E-10; // dokladnosc
	static int s_iNext[3] = {2, 3, 1};
	int i=0;
	int j,k;
	double fRoot;
	double *tmp[3] = {&eps1, &eps2, &eps3};

	tr=matrix_m[0][0] + matrix_m[1][1] + matrix_m[2][2];

	eta = 0.5*sqrt(tr+1);

	if(eta>EPSILON)
	{
		eps1 = (matrix_m[2][1] - matrix_m[1][2]) / (4*eta);
		eps2 = (matrix_m[0][2] - matrix_m[2][0]) / (4*eta);
		eps3 = (matrix_m[1][0] - matrix_m[0][1]) / (4*eta);
	}
	else
	{
		if (matrix_m[1][1] > matrix_m[0][0]) i=1;
		if (matrix_m[2][2] > matrix_m[1][1]) i=2;

		j = s_iNext[i-1];
		k = s_iNext[j-1];

		fRoot = sqrt(matrix_m[i-1][i-1] - matrix_m[j-1][j-1] - matrix_m[k-1][k-1] + 1.0);

		*tmp[i-1] = 0.5*fRoot;
		fRoot = 0.5/fRoot;
		eta = (matrix_m[k-1][j-1] - matrix_m[j-1][k-1])*fRoot;
		*tmp[j-1] = (matrix_m[j-1][i-1] + matrix_m[i-1][j-1])*fRoot;
		*tmp[k-1] = (matrix_m[k-1][i-1] + matrix_m[i-1][k-1])*fRoot;
	}

	// Przepisanie wyniku do tablicy

	for(i=0; i<3; i++) t[i] = matrix_m[3][i];
	t[3]=eta;
	t[4]=eps1;
	t[5]=eps2;
	t[6]=eps3;
} //void Homog_matrix::get_xyz_quaternion(double t[7]) const

// Ustawienie elementu macierzy.
void Homog_matrix::set_value(int i, int j, const double value)
{
	matrix_m[i][j] = value;
}//: set_value

// Zwrocenie elementu macierzy.
void Homog_matrix::get_value(int i, int j, double &value) const
{
	value = matrix_m[i][j];
}//: get_value

// Zwrocenie elementu macierzy.
double Homog_matrix::get_value(int i, int j) const
{
	return matrix_m[i][j];
}//: get_value


// operator przypisania
Homog_matrix & Homog_matrix::operator=(const Homog_matrix & wzor)
{
	if(this == &wzor) return *this;
	set_from_frame_tab(wzor.matrix_m);
	return *this;
}// end Homog_matrix::operator=(const Homog_matrix & wzor)


Homog_matrix Homog_matrix::operator* (const Homog_matrix & m) const
{
// mnozenie macierzy
// mnozenie realizowane jest zgodnie ze wzorem 2.41 ze strony 54
// ksiazki: "Wprowadzenie do robotyki" John J. Craig


	Homog_matrix zwracana;
	int i, j;			// i - i-ta kolumna
					// j - j-ty wiersz

	// macierz rotacji
	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
		{
			for(int a=0; a<3; a++)
			{
				if(a == 0)
					zwracana.matrix_m[j][i] = matrix_m[j][a] * m.matrix_m[a][i];
				else
					zwracana.matrix_m[j][i] += matrix_m[j][a] * m.matrix_m[a][i];

			}
		}

	// wektor przesuniecia
	for(j =0; j<3; j++)
	{
		for(i=0; i<3; i++)
			zwracana.matrix_m[j][3] += matrix_m[j][i] * m.matrix_m[i][3];
		zwracana.matrix_m[j][3] += matrix_m[j][3];
	}

	return zwracana;
}// end Homog_matrix::operator* (const Homog_matrix & m) const


void Homog_matrix::operator *= (const Homog_matrix & m)
{
     // mnozenie macierzy
	// mnozenie realizowane jest zgodnie ze wzorem 2.41 ze strony 54
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	int i, j;			// i - i-ta kolumna
					// j - j-ty wiersz
	frame_tab tymcz_m;
	this->get_frame_tab(tymcz_m);

	// macierz rotacji
	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
		{
			for(int a=0; a<3; a++)
			{
				if(a == 0)
					matrix_m[j][i] = tymcz_m[j][a] * m.matrix_m[a][i];
				else
					matrix_m[j][i] += tymcz_m[j][a] * m.matrix_m[a][i];
			}
		}

	// wektor przesuniecia
	for(j =0; j<3; j++)
	{
		for (i=0; i<3; i++)
		{
            if(i == 0)
		         matrix_m[j][3] = 0;
			matrix_m[j][3] += tymcz_m[j][i] * m.matrix_m[i][3];
         }
        matrix_m[j][3] += tymcz_m[j][3];
	}

}// end Homog_matrix::operator *= (const Homog_matrix & m)

K_vector Homog_matrix::operator*(const K_vector & w) const
{
	// operator mnozenia
	// umozliwia otrzymanie wektora w ukladzie odniesienia reprezentowanym przez macierz jednorodna
	// argument: obiekt klasy K_vector

	K_vector zwracany;
	int i;
	int j;
	// i - i-ta kolumna
	// j - j-ty wiersz

	for(j=0;j<3;j++)
		for(i=0;i<3;i++)
			zwracany.w[j] += matrix_m[j][i] * w.w[i];

	for(i=0;i<3;i++)
		zwracany.w[i] +=matrix_m[i][3];

	return zwracany;

}// end Homog_matrix::operator*(const K_vector & w) const

K_vector Homog_matrix::operator*(const double tablica[3]) const
{
	// operator mnozenia
	// umozliwia otrzymanie wektora w ukladzie odniesienia reprezentowanym przez macierz jednorodna
	// argument: tablica trzyelementowa reprezentujaca wektor

	K_vector zwracany;
	int i;
	int j;
	// i - i-ta kolumna
	// j - j-ty wiersz

	for(j=0;j<3;j++)
		for(i=0;i<3;i++)
			zwracany.w[j] += matrix_m[j][i] * tablica[i];

	for(i=0;i<3;i++)
		zwracany.w[i] +=matrix_m[i][3];

return zwracany;

}// end Homog_matrix::operator*(const double tablica[3]) const

Homog_matrix Homog_matrix::operator!() const
{
	// przeksztalcenie odwrotne
	// odwrocenie macierzy zgodnie ze wzorem 2.45 ze strony 55
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	Homog_matrix zwracana;
	int i, j;


	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			zwracana.matrix_m[i][j] = matrix_m[j][i];

	for(j=0; j<3; j++)
		for(i=0; i<3; i++)
			zwracana.matrix_m[j][3] += -1*zwracana.matrix_m[j][i] * matrix_m[i][3];

return zwracana;
}// end Homog_matrix::operator!()


int Homog_matrix::operator==(const Homog_matrix & comp) const
{

	// opeartor porownania - sprawdza czy dwa obiekty Homog_matrix sa takie same
	// warunki:
	// obie macierze musza spelniac warunek jednorodnosci
	// 	lub obie go nie spelniac
	// suma kwadratow elementow macierzy musi byc mniejsza od eps

	// zwraca:
	// 1 - macierze rowne
	// 0 - macierzy rozne

	// i - i-ta kolumna
	// j - j-ty wiersz

	const double eps = 1e-5;
	double val = 0;

	if(this->is_valid() != comp.is_valid())
		return(0);

	Homog_matrix T;
	Homog_matrix A(*this);
	Homog_matrix B(comp);

	T = A * !B;

	frame_tab t_m;
	T.get_frame_tab(t_m);

	for(int i=0; i<4;i++)
		for(int j=0; j<3; j++)
		{
			if(i == j)
				val += ((t_m[i][i]-1) * (t_m[i][i]-1));
			else
				val += (t_m[j][i] * t_m[j][i]);
		}

	if(val > eps)
		return(0);			// przekroczony eps
							// macierze sa rozne


return(1);
}// end int Homog_matrix::operator==(const Homog_matrix & comp) const



int Homog_matrix::operator!=(const Homog_matrix & comp) const
{
	// operator porownania - sprawdza czy porownywane obiekty klasy Homog_matrix sa rozne od siebie

	// zwraca:
	// 1 - macierze rozne
	// 0 - macierze rowne

	const double eps = 1e-5;
	double val = 0;

	if(this->is_valid() != comp.is_valid())
		return(1);

	Homog_matrix T;
	Homog_matrix A(*this);
	Homog_matrix B(comp);

	T = A * !B;

	frame_tab t_m;
	T.get_frame_tab(t_m);

	for(int i=0; i<4;i++)
		for(int j=0; j<4; j++)
		{
			if(i == j)
				val += ((t_m[i][i]-1) * (t_m[i][i]-1));
			else
				val += (t_m[j][i] * t_m[j][i]);
		}

	if(val > eps)
		return(1);			// przekroczony eps
							// macierze sa rozne


return(0);
}// end int Homog_matrix::operator!=(const Homog_matrix & comp) const

double* Homog_matrix::operator[](const int i)
{
        //    printf("RHS a[%2d]\n", i );
        return matrix_m[i];
}

std::ostream&  operator<<(std::ostream & strumien, Homog_matrix & m)
{
	// operator wypisania
	// przedstawia macierz jednorodna w przyjaznej dla czlowieka formie

	for(int j=0; j<3; j++)
	{
		for(int i=0; i<4; i++)
		{
			strumien << m.matrix_m[j][i] << "\t\t";
		}
		strumien << std::endl;
	}
	strumien << "0\t\t0\t\t0\t\t1\t\t\n";

return strumien;
}// end operator<<(std::ostream & strumien, Homog_matrix & m)


int Homog_matrix::is_valid() const
{
	// funkcja sprawdza czy macierz jest macierza jednorodna
	// jesli tak jest to funkcja zwraca 1
	// w przeciwnym wypadku funkcja zwraca 0

    const double eps = 1e-5;
    int i,j,k; double value;

    // obliczenia zgodne ze wzorem 6.12 z punktu 6.3.1 pracy:
	// Biblioteka funkcji matematycznych wspomagajacych sterowanie robotami
    for(i=0; i<3; i++)
    {
          value = 0;
          for(k = 0; k<3; k++)
                value += (matrix_m[i][k] * matrix_m[i][k]);
          if ((value > 1+eps) || (value < 1-eps))
             return(0);
    }

	// obliczenia zgodne ze wzorem 6.13 z punktu 6.3.1 pracy:
	// Biblioteka funkcji matematycznych wspomagajacych sterowanie robotami
    for(i=0;i<3;i++)
    {
       j = ((i+1)%3);
       value = 0;
       for(k=0; k<3; k++)
                value += (matrix_m[i][k] * matrix_m[j][k]);
       if ((value < -eps) || (value > eps))
          return(0);
    }

return(1);
}// end Homog_matrix::is_valid() const


// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
void Homog_matrix::set_translation_vector(double x, double y, double z)
{
	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;
}//: set_translation_vector


// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
void Homog_matrix::set_translation_vector(const K_vector xyz)
{
	matrix_m[0][3] = xyz[0];
	matrix_m[1][3] = xyz[1];
	matrix_m[2][3] = xyz[2];
}//: set_translation_vector

// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
void Homog_matrix::set_translation_vector(double t[3])
{
	matrix_m[0][3] = t[0];
	matrix_m[1][3] = t[1];
	matrix_m[2][3] = t[2];
};//: set_translation_vector


void Homog_matrix::set_translation_vector(const Homog_matrix &wzor)
{
	matrix_m[0][3] = wzor.matrix_m[0][3];
	matrix_m[1][3] = wzor.matrix_m[1][3];
	matrix_m[2][3] = wzor.matrix_m[2][3];
}

// wyzerowanie wektora translacji.
void Homog_matrix::remove_translation()
{
	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;
}//: set_translation_vector


	// wstawienie jedynek na diagonalii rotacji
void Homog_matrix::remove_rotation()
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}
	}
}//: set_translation_vector


Homog_matrix Homog_matrix::return_with_with_removed_translation() const
{

	// i - i-ta kolumna
	// j - j-ty wiersz

	Homog_matrix zwracana;
	int i, j;


	for(i=0; i<3; i++)
		for(j=0; j<3; j++)
			zwracana.matrix_m[i][j] = matrix_m[i][j];

	zwracana.matrix_m[0][3] = 0;
	zwracana.matrix_m[1][3] = 0;
	zwracana.matrix_m[2][3] = 0;

	return zwracana;
}// end Homog_matrix::operator!()


Homog_matrix Homog_matrix::return_with_with_removed_rotation() const
{

	// i - i-ta kolumna
	// j - j-ty wiersz

	Homog_matrix zwracana;

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(i == j)
				zwracana.matrix_m[i][j] = 1;
			else
				zwracana.matrix_m[i][j] = 0;
		}
	}

	zwracana.matrix_m[0][3] = matrix_m[0][3];
	zwracana.matrix_m[1][3] = matrix_m[1][3];
	zwracana.matrix_m[2][3] = matrix_m[2][3];

	return zwracana;

}// end Homog_matrix::operator!()


// Zwraca obecny wektor translacji.
void Homog_matrix::get_translation_vector(double t[3]) const
{
	t[0] = matrix_m[0][3];
	t[1] = matrix_m[1][3];
	t[2] = matrix_m[2][3];
}//: get_translation_vector


// Ustawienie macierzy rotacji. Wektor translacji pozostaje niezmieniony.
void Homog_matrix::set_rotation_matrix(double r[3][3])
{
	for(int j=0; j<3; j++)
		for(int i=0; i<3; i++)
			matrix_m[j][i] = r[j][i];
}//: set_rotation_matrix


void Homog_matrix::set_rotation_matrix(const Homog_matrix &wzor)
{
	for(int j=0; j<3; j++)
		for(int i=0; i<3; i++)
			matrix_m[j][i] = wzor.matrix_m[j][i];
}//: set_rotation_matrix

// Pobranie macierzy rotacji.
void Homog_matrix::get_rotation_matrix(double r[3][3]) const
{
	for(int j=0; j<3; j++)
		for(int i=0; i<3; i++)
			r[j][i] = matrix_m[j][i];
}//: get_rotation_matrix


} // namespace lib
			} // namespace mrrocpp

