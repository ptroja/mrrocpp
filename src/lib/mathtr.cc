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
#include "lib/mathtr.h"

namespace mrrocpp {
namespace lib {

// Sprowadzenie wartosci kata do przedzialu (-M_PI, M_PI >.
double reduce (double angle)
{
  while (angle > M_PI)
	angle-=2*M_PI;

  while (angle <= -M_PI)
	angle+=2*M_PI;

 return(angle);
}// end reduce


// Sprowadzenie wartosci kata do przedzialu (min, max>.
double reduce(double angle, double min, double max, double offset)
{
  while (angle >= max)
	angle-= offset;

  while (angle < min)
	angle+=offset;

 return(angle);
}// end reduce

// ******************************************************************************************
//                                               definicje skladowych klasy VECTOR
// ******************************************************************************************
K_vector::K_vector()
{
	// konstruktor domniemany
	// tworzy wektor: [0, 0, 0]

	for(int i=0; i<3; i++)
		w[i] = 0;

}// end K_vector::K_vector()

K_vector::K_vector(double t[3])
{
	// utworzenie wektora o wspolrzednych okreslonych przez podana jako argument tablice

	for(int i=0; i<3; i++)
		w[i] = t[i];

}// end K_vector::K_vector(double t[3])

K_vector::K_vector(double x, double y, double z)			// utworzenie wektora na podstawie trzech wsp.
{
	w[0] = x;	w[1] = y; w[2] = z;
}


K_vector::K_vector(const K_vector & wzor)
{
	// konstruktor kopiujacy
	// tworzy wektor o takich samych wspolrzednych jak wektor podany jako argument

	for(int i=0; i<3; i++)
		w[i] = wzor.w[i];

}// end K_vector::K_vector(const K_vector & wzor)


// Ustawienie elementu wektora.
void K_vector::set_value(int i, const double value)
{
	w[i] = value;
}

// Zwrocenie elementu wektora.
void K_vector::get_value(int i, double &value) const
{
	value = w[i];
}

// Zwrocenie dligosci wektora.
double K_vector::get_length() const
{
	return sqrt (w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
}


// Zwrocenie dligosci wektora.
void K_vector::normalize()
{
	double length = get_length();
	w[0] /= length;
	w[1] /= length;
	w[2] /= length;
}


// Zwrocenie elementu wektora.
double K_vector::get_value(int i) const
{
	return w[i];
}


void K_vector::to_table(double tablica[3]) const
{
	// przepisanie wespolrzednych wektora do tablicy trzyelementowej

	for(int i=0; i<3;i++)
		tablica[i]=w[i];

}// end K_vector::to_table(double tablica[3]) const

K_vector & K_vector::operator=(const double tablica[3])
{
	// przepisanie tablicy trzyelementowej wspolrzednych do wektora

	for(int i=0; i<3; i++) w[i]=tablica[i];
	return *this;
}// end K_vector::operator=(const double tablica[3]) const

K_vector & K_vector::operator=(const K_vector & p)
{
	// operator przypisania

	if(this == &p) return *this;

	for(int i=0; i<3; i++)
		w[i] = p.w[i];

return *this;
}// end K_vector::operator=(const K_vector & p)

K_vector K_vector::operator+(const K_vector & dod) const
{
	// operator realizujacy dodawanie wektorow

	K_vector zwracany;
	for(int i=0; i<3; i++) zwracany.w[i] = w[i] + dod.w[i];
	return zwracany;
}// end K_vector::operator+(const K_vector & dod) const

K_vector K_vector::operator-(const K_vector &odejmowany) const
{
	// operator realizujacy 	odejmowanie wektorow

	K_vector zwracany;
	for(int i=0; i<3; i++) zwracany.w[i] = w[i] - odejmowany.w[i];
	return zwracany;
}// end K_vector::operator-(const K_vector & dod) const


K_vector K_vector::operator*(const K_vector & iloczyn) const
{
	// operator realizujacy iloczyn wektorowy 2 wektor�w 3x1

	K_vector zwracany;
	zwracany.w[0] = w[1] * iloczyn.w[2] - w[2] * iloczyn.w[1];
	zwracany.w[1] = w[2] * iloczyn.w[0] - w[0] * iloczyn.w[2];
	zwracany.w[2] = w[0] * iloczyn.w[1] - w[1] * iloczyn.w[0];
	return zwracany;
}// end K_vector::operator*(const K_vector & dod) const

K_vector K_vector::operator*(const double skalar) const
{
	// operator realizujacy iloczyn skalarny wektora przez liczbe

    K_vector zwracany;
	for(int i=0;i<3;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}// end K_vector K_vector::operator*(const double skalar) const

void K_vector::operator*=(const double skalar)
{
	// operator realizujacy iloczyn skalarny wektora przez liczbe

	for(int i=0;i<3;i++) w[i] *= skalar;

}// end void K_vector::operator*=(const double skalar)



void K_vector::operator+=(const K_vector & dod)
{
	// operator laczacy w sobie dzialanie dwoch operatorow:
	// - dodawania
	// - przypisania

	for(int i=0; i<3; i++)
		w[i] += dod.w[i];

}// end K_vector::operator+=(const K_vector & dod)

std::ostream& operator<<(std::ostream & s, K_vector & w)
{
	// operator wypisania
	// wypisuje wspolrzedne wektora w przyjaznej dla czlowieka formie

	s << "[ ";
	for(int i=0; i<3; i++)
		s << w.w[i] << " ";
	s << "]\n";
return s;

}// end operator<<(std::ostream & s, K_vector & w)


// in theory, the RHS operator
const double K_vector::operator[](const int i ) const
{
	//    printf("RHS a[%2d]\n", i );
	return w[i];
}

// in theory, the LHS operator
double& K_vector::operator[](const int i )
{
	// printf("LHS a[%2d]\n", i );
	return w[i];
}



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

	for(int i=0; i<4; i++)
		for(int j=0; j<3; j++)
		{
			if(i == j)
				matrix_m[j][i] = 1;
			else
				matrix_m[j][i] = 0;
		}
};//: Homog_matrix()


Homog_matrix::Homog_matrix(K_vector versor_x, K_vector versor_y, K_vector versor_z, K_vector angles)
{
	matrix_m[0][0] = 1;
	matrix_m[1][0] = versor_x[2]*angles[0]+versor_y[2]*angles[1]+versor_z[2]*angles[2];
	matrix_m[2][0] = -1*(versor_x[1]*angles[0]+versor_y[1]*angles[1]+versor_z[1]*angles[2]);

	matrix_m[0][1] = -1*(versor_x[2]*angles[0]+versor_y[2]*angles[1]+versor_z[2]*angles[2]);
	matrix_m[1][1] = 1;
	matrix_m[2][1] = versor_x[0]*angles[0]+versor_y[0]*angles[1]+versor_z[0]*angles[2];

	matrix_m[0][2] = versor_x[1]*angles[0]+versor_y[1]*angles[1]+versor_z[1]*angles[2];
	matrix_m[1][2] = -1*(versor_x[0]*angles[0]+versor_y[0]*angles[1]+versor_z[0]*angles[2]);
	matrix_m[2][2] = 1;

	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;

}//: Homog_matrix(K_vector versor_x, K_vector versor_y, K_vector versor_z, double angles[3])


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

}//: Homog_matrix(K_vector versor_x, K_vector versor_y, K_vector versor_z, double angles[3])


Homog_matrix::Homog_matrix(double r[3][3], double t[3])
{
	// utworznie macierzy jednorodnej na podstawie:
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
}//: Homog_matrix(double r[3][3], double t[3])


// Utworzenie macierzy jednorodnej na podstawie zawartosci tablicy podanej jako argument.
Homog_matrix::Homog_matrix(const frame_tab frame)
{
	set_frame_tab(frame);
}//: Homog_matrix::Homog_matrix(frame_tab frame)


// kontruktor kopiujacy
// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
Homog_matrix::Homog_matrix(const Homog_matrix &wzor)
{
	set_frame_tab(wzor.matrix_m);
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
Homog_matrix::Homog_matrix (double r11, double r12, double r13, double t1, double r21, double r22, double r23, double t2, double r31, double r32,
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
	set_xyz_angle_axis(kx, ky, kz, alfa, x, y, z);
}//: Homog_matrix::Homog_matrix(double kx, double ky, double kz, double alfa, double x, double y, double z)


// Konstruktor, ktory wypelnienia wspolczynniki macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
Homog_matrix::Homog_matrix (POSE_SPECIFICATION mtr_ps, const K_vector axis_with_angle, const K_vector translation)
{
	switch (mtr_ps)
	{
		case MTR_XYZ_ANGLE_AXIS:
			set_xyz_angle_axis (axis_with_angle, translation);
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
			set_xyz_angle_axis (translation_and_axis_with_angle);
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
			set_xyz_angle_axis (t);
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
		case MTR_XYZ_EULER_ZYZ:
			set_xyz_euler_zyz (x, y, z, alfa, beta, gamma);
			break;
		case MTR_MECH_XYZ_EULER_ZYZ:
			set_mech_xyz_euler_zyz (x, y, z, alfa, beta, gamma);
			break;

		case MTR_XYZ_RPY:
			set_xyz_rpy (x, y, z, alfa, beta, gamma);
			break;
		case MTR_XYZ_ANGLE_AXIS:
			set_xyz_angle_axis (alfa, beta, gamma, x, y, z);
			break;
		default:
			break;
	}

} //: Homog_matrix();


// Zwrocenie obecnej tablicy, zawierajacej dane macierzy jednorodnej.
void Homog_matrix::get_frame_tab(frame_tab frame) const
{
	copy_frame(frame, matrix_m);
}//: get_frame_tab


// Ustawienie tablicy, ktora zawiera dane macierzy jednorodnej.
void Homog_matrix::set_frame_tab(const frame_tab frame)
{
	copy_frame(matrix_m, frame);
}//: set_frame_tab



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
void Homog_matrix::get_xyz_euler_zyz(double t[6]) const
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

	};//: else

	// Przepisanie wyniku do tablicy
	for(int i = 0;i<3;i++)
		t[i] = matrix_m[i][3];
	t[3] = alfa;
	t[4] = beta;
	t[5] = gamma;
}//: get_xyz_euler_zyz

// Przeksztalcenie do formy XYZ_EULER_ZYZ dla robota IRP_6 MECHATRONIKAi zwrocenie w tablicy.
void Homog_matrix::get_mech_xyz_euler_zyz(double t[6]) const
{

	// Przepisanie wyniku do tablicy
	for(int i = 0;i<3;i++)
		t[i] = matrix_m[i][3];
	t[3] = matrix_m[0][0];
	t[4] = matrix_m[0][1];
	t[5] = matrix_m[0][2];
}//: get_mech_xyz_euler_zyz

void Homog_matrix::set_xyz_euler_zyz(const double t[6])
{
	set_xyz_euler_zyz(t[0], t[1], t[2], t[3], t[4], t[5]);
}



// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_EULER_ZYZ.
void Homog_matrix::set_xyz_euler_zyz(double x, double y, double z, double alfa, double beta, double gamma)
{
	// alfa, beta, gamma - Katy Euler'a Z-Y-Z
	double c_alfa, s_alfa, c_beta, s_beta, c_gamma, s_gamma;

	// Zredukowanie katow.
	alfa = reduce(alfa, -M_PI, M_PI, 2*M_PI);
	beta = reduce(beta, 0, M_PI, M_PI);
	gamma = reduce(gamma, -M_PI, M_PI, 2*M_PI);

	// Obliczenie sinusow/cosinusow.
	c_alfa = cos(alfa);
	s_alfa = sin(alfa);
	c_beta = cos(beta);
	s_beta = sin(beta);
	c_gamma= cos(gamma);
	s_gamma= sin(gamma);

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
	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;

}//: set_xyz_euler_zyz

// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_EULER_ZYZ dla robota IRP-6_MECHATRONIKA.
void Homog_matrix::set_mech_xyz_euler_zyz(double x, double y, double z, double alfa, double beta, double gamma)
{
	// alfa, beta, gamma - Katy Euler'a Z-Y-Z
	double c_alfa, s_alfa, c_beta, s_beta, c_gamma, s_gamma;

	// Zredukowanie katow.
	alfa = reduce(alfa, -M_PI, M_PI, 2*M_PI);
	beta = reduce(beta, 0, M_PI, M_PI);
	gamma = reduce(gamma, -M_PI, M_PI, 2*M_PI);

	// Obliczenie sinusow/cosinusow.
	c_alfa = cos(alfa);
	s_alfa = sin(alfa);
	c_beta = cos(beta);
	s_beta = sin(beta);
	c_gamma= cos(gamma);
	s_gamma= sin(gamma);

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
	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;
}//: set_mech_xyz_euler_zyz


// notacja i wzory z Craiga - wydanie angielskie str. 41

// UWAGA ponizsze dwie funckje nie byly testowane - po pozytywnych  testach usunac komentarz
// Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
void Homog_matrix::get_xyz_rpy(double t[6]) const
{
	// x, y, z
	t[0] = matrix_m[0][3];
	t[1] = matrix_m[1][3];
	t[2] = matrix_m[2][3];

	// alfa (wokol z) , beta (wokol y), gamma (wokol x)
	t[3] = atan2 (matrix_m[2][1], matrix_m[2][2]);
	t[4] = atan2 (matrix_m[2][0], sqrt (matrix_m[0][0]*matrix_m[0][0] + matrix_m[1][0]*matrix_m[1][0]));
	t[5] = atan2 (matrix_m[1][0], matrix_m[0][0]);
}


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
void Homog_matrix::set_xyz_rpy(double x, double y, double z, double alfa, double beta, double gamma)
{
	// alfa (wokol z) , beta (wokol y), gamma (wokol x)
	double c_alfa, s_alfa, c_beta, s_beta, c_gamma, s_gamma;

	c_alfa = cos(alfa);
	s_alfa = sin(alfa);
	c_beta = cos(beta);
	s_beta = sin(beta);
	c_gamma = cos(gamma);
	s_gamma = sin(gamma);


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
	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;
}



// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_xyz_angle_axis(double kx, double ky, double kz, double alfa, double x, double y, double z)
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	double c_alfa, s_alfa, v_alfa;		// c_alfa - kosinus kata alfa
											// s_alfa - sinus kata alfa
											// v_alfa = 1 - c_alfa;

	// wartosci poszczegolnych funkcji trygonometrycznych dla kata obrotu
	c_alfa = cos(alfa);
	s_alfa = sin(alfa);
	v_alfa = 1 - c_alfa;

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

}//: set_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS - kat wliczony w dlugos osi.
void Homog_matrix::set_xyz_angle_axis(double kx, double ky, double kz, double x, double y, double z)
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okreslona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	double alfa =sqrt(kx*kx + ky*ky +kz*kz);

	double kx_l,  ky_l, kz_l;

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

	set_xyz_angle_axis (kx_l, ky_l, kz_l, alfa, x, y, z);

}//: set_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_xyz_angle_axis(const K_vector axis_with_angle, const K_vector translation)
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	set_xyz_angle_axis (axis_with_angle[0], axis_with_angle[1], axis_with_angle[2], translation[0], translation[1], translation[2]);

}//: set_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_xyz_angle_axis(const Ft_v_vector translation_and_axis_with_angle)  // kat wliczony w os
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	set_xyz_angle_axis (translation_and_axis_with_angle[3], translation_and_axis_with_angle[4], translation_and_axis_with_angle[5],
		 translation_and_axis_with_angle[0], translation_and_axis_with_angle[1], translation_and_axis_with_angle[2]);

}//: set_xyz_angle_axis


// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_ANGLE_AXIS.
void Homog_matrix::set_xyz_angle_axis(const double t[6])  // kat wliczony w os
{
	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	set_xyz_angle_axis (t[3], t[4], t[5], t[0], t[1], t[2]);

}//: set_xyz_angle_axis

//Wypelnienie macierzy na podstawie parametrow  wejsciowych w postaci kwaternionu
void Homog_matrix::set_xyz_quaternion(double eta, double eps1, double eps2, double eps3, double x, double y, double z)
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

}// Homog_matrix::set_xyz_quaternion(double eta, K_vector eps, double x, double y, double z)


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


// Przeksztalcenie do formy XYZ_ANGLE_AXIS i zwrocenie w tablicy.
void Homog_matrix::get_xyz_angle_axis(double t[6]) const
{
	// przeksztalcenie macierzy jednorodnej do rozkazu w formie XYZ_ANGLE_AXIS
	const double EPS = zero_eps;
	const double delta = delta_m;
	double alfa;		// kat obrotu
	double s_alfa;	// sinus kata obrotu alfa
	int i;				// licznik petli
	double Kd[3];	// Kd - K z "daszkiem" - wersor kierunkowy

	// obliczenia zgodne ze wzorami 2.81 i 2.82 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	double value = (matrix_m[0][0]+matrix_m[1][1]+	matrix_m[2][2]-1)/2;

	// wyeliminowanie niedokladnosci obliczeniowej lub bledu programisty
	if(value < -1)
		value = -1;
	else if (value > 1)
		value = 1;

	alfa = acos(value);

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

		for(i=0; i<3; i++)
			Kd[i] = 0;

		alfa = 0;

	}
	else																				// standardowe obliczenia
	{
		s_alfa = sin(alfa);

		Kd[0] = (1/(2*s_alfa))*(matrix_m[2][1] - matrix_m[1][2]);
		Kd[1] = (1/(2*s_alfa))*(matrix_m[0][2] - matrix_m[2][0]);
		Kd[2] = (1/(2*s_alfa))*(matrix_m[1][0] - matrix_m[0][1]);
	}

	// Przepisanie wyniku do tablicy
	for(i=0;i<3;i++)
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
	set_frame_tab(wzor.matrix_m);
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

	double eps = 1e-5;
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

	double eps = 1e-5;
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


// ******************************************************************************************
//                                           definicje skladowych klasy Ft_v_vector
// ******************************************************************************************

Ft_v_vector::Ft_v_vector()
{
	// konstruktor domniemany
	// tworzy wektor: [0, 0, 0, 0, 0, 0]

	memset (w, 0, sizeof(w));

}// end Ft_v_vector::Ft_v_vector()

Ft_v_vector::Ft_v_vector(const double t[6])
{
	// utworznie wektora o parametrach podanych w tablicy, bedacej argumentem
	set_values(t);

}// end Ft_v_vector::Ft_v_vector(double t[6])

Ft_v_vector::Ft_v_vector(double fx, double fy, double fz, double tx, double ty, double tz)
{
	// utworznie wektora o parametrach podanych w tablicy, bedacej argumentem
	set_values(fx, fy, fz, tx, ty, tz);

}// end Ft_v_vector::Ft_v_vector(double t[6])


Ft_v_vector::Ft_v_vector(const K_vector force, const K_vector torque)
{
	for(int i=0; i<3; i++)
	{
		w[i] = force[i];
		w[i+3] = torque[i];
	}
}

/* ------------------------------------------------------------------------
Wyznaczenie uchybu pozycji tzn. r�znicy pozycji aktualnej i zadanej

  Wejscie:
  * local_current_end_effector_frame - macierz przeksztacenia jednorodnego (MPJ)
		opisujca aktualne poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.
  * local_desired_end_effector_frame - macierz przeksztacenia jednorodnego (MPJ)
		opisujca zadane poloenie i orientacje koncowki (narzedzia) w ukladzie bazowym.

  Wyjscie:
  * desired_distance - 6-elementowy wektor opisujacy uchyb. 3 pierwsze elementy odpowiadaj�
		przesuni�ciu liniowym, a kolejne 3 uchybowi rotacji

Sibi
 ------------------------------------------------------------------------ */

void Ft_v_vector::position_distance(frame_tab* local_current_end_effector_frame, frame_tab* local_desired_end_effector_frame)
{

double n_t[3], n_d[3], o_t[3], o_d[3], a_t[3], a_d[3];

//Wyliczenie wektora przesuniecia : w - predkosc katowa
// n, o, a - wektory normalny, orientacji i zbli�enia

n_t[0]=(*local_current_end_effector_frame)[0][0];
n_t[1]=(*local_current_end_effector_frame)[1][0];
n_t[2]=(*local_current_end_effector_frame)[2][0];

n_d[0]=(*local_desired_end_effector_frame)[0][0];
n_d[1]=(*local_desired_end_effector_frame)[1][0];
n_d[2]=(*local_desired_end_effector_frame)[2][0];

o_t[0]=(*local_current_end_effector_frame)[0][1];
o_t[1]=(*local_current_end_effector_frame)[1][1];
o_t[2]=(*local_current_end_effector_frame)[2][1];

o_d[0]=(*local_desired_end_effector_frame)[0][1];
o_d[1]=(*local_desired_end_effector_frame)[1][1];
o_d[2]=(*local_desired_end_effector_frame)[2][1];

a_t[0]=(*local_current_end_effector_frame)[0][2];
a_t[1]=(*local_current_end_effector_frame)[1][2];
a_t[2]=(*local_current_end_effector_frame)[2][2];

a_d[0]=(*local_desired_end_effector_frame)[0][2];
a_d[1]=(*local_desired_end_effector_frame)[1][2];
a_d[2]=(*local_desired_end_effector_frame)[2][2];

//Wyliczenie wektora przesuniecia : v - predkosc obrotowa

w[0]=(0.5)*((n_t[1]*n_d[2]-n_t[2]*n_d[1])+(o_t[1]*o_d[2]-o_t[2]*o_d[1])+(a_t[1]*a_d[2]-a_t[2]*a_d[1]));
w[1]=(0.5)*((n_t[2]*n_d[0]-n_t[0]*n_d[2])+(o_t[2]*o_d[0]-o_t[0]*o_d[2])+(a_t[2]*a_d[0]-a_t[0]*a_d[2]));
w[2]=(0.5)*((n_t[0]*n_d[1]-n_t[1]*n_d[0])+(o_t[0]*o_d[1]-o_t[1]*o_d[0])+(a_t[0]*a_d[1]-a_t[1]*a_d[0]));

//Wyliczenie wektora przesuniecia : v - predkosc liniowa

w[3]=(*local_desired_end_effector_frame)[0][3]-(*local_current_end_effector_frame)[0][3];
w[4]=(*local_desired_end_effector_frame)[1][3]-(*local_current_end_effector_frame)[1][3];
w[5]=(*local_desired_end_effector_frame)[2][3]-(*local_current_end_effector_frame)[2][3];
}// end Ft_v_vector::position_distance(frame_tab*, frame_tab* )

K_vector Ft_v_vector::get_force_K_vector() const
{
	K_vector tmp(w[0], w[1], w[2]);
	return tmp;
}


K_vector Ft_v_vector::get_torque_K_vector() const
{
	K_vector tmp(w[3], w[4], w[5]);
	return tmp;
}


Ft_v_vector::Ft_v_vector(const Ft_v_vector & wzor)
{
	// konstruktor kopiujacy
	// tworzy wektor, ktorego parametry przyjmuja wartosci takie jak parametry wektora podanego jako argument

	set_values(wzor.w);

}// end Ft_v_vector::Ft_v_vector(const Ft_v_vector & wzor)


// Ustawienie elementu wektora.
void Ft_v_vector::set_values(const double t[6])
{
	memcpy(w, t, sizeof(w));
}

// Ustawienie elementu wektora.
void Ft_v_vector::set_values(double fx, double fy, double fz, double tx, double ty, double tz)
{
	w[0] = fx; w[1] = fy; w[2] = fz;
	w[3] = tx; w[4] = ty; w[5] = tz;
}

// Ustawienie elementu wektora.
void Ft_v_vector::set_value(int i, const double value)
{
	w[i] = value;
}

// Zwrocenie elementu wektora.
void Ft_v_vector::get_value(int i, double &value) const
{
	value = w[i];
}


// Zwrocenie elementu wektora.
double Ft_v_vector::get_value(int i) const
{
	return w[i];
}

// Wyspisanie na ekran wektora
void Ft_v_vector::wypisz_wartosc_na_konsole() const
{
	printf("% 7.3f, % 7.3f, % 7.3f, % 7.3f, % 7.3f, % 7.3f\n", w[0], w[1], w[2], w[3], w[4], w[5]);
}


 // in theory, the RHS operator
  const double Ft_v_vector::operator[](const int i ) const
  {
//    printf("RHS a[%2d]\n", i );
    return w[i];
  }

  // in theory, the LHS operator
  double& Ft_v_vector::operator[](const int i )
  {
   // printf("LHS a[%2d]\n", i );
    return w[i];
  }



Ft_v_vector & Ft_v_vector::operator=(const Ft_v_vector & p)
{
	// operator przypisania
	// parametry wektora przyjmuja wartosci takie jak parametry wektora podanego jako argument

	if(this == &p) return *this;

	set_values(p.w);

return *this;
}// end Ft_v_vector::operator=(const Ft_v_vector & p)

Ft_v_vector Ft_v_vector::operator+(const Ft_v_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Ft_v_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] + dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const


Ft_v_vector Ft_v_vector::operator-(const Ft_v_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Ft_v_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] - dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const



Ft_v_vector Ft_v_vector::operator*(const double skalar) const
{
	Ft_v_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}


Ft_v_vector Ft_v_vector::operator!() const
{
	// odwrocenie wektora

	Ft_v_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


Ft_v_vector Ft_v_vector::operator-() const
{
	// odwrocenie wektora

	Ft_v_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


void Ft_v_vector::operator+=(const Ft_v_vector & dod)
{

	for(int i=0; i<6; i++)
		w[i] += dod.w[i];

}// end Ft_v_vector::operator+=(const Ft_v_vector & dod)

void Ft_v_vector::to_table(double tablica[6]) const
{
	// przepisanie parametrow wektora do szescioelementowej tablicy podanej jako argument

	for(int i=0; i<6;i++)
		tablica[i]=w[i];

}// end Ft_v_vector::to_table(double tablica[6]) const

std::ostream& operator<<(std::ostream & s, Ft_v_vector & w)
{
	// operator wypisania
	// przedstawia wektor w przyjaznej dla czlowieka formie

	s << "[ ";
	for(int i=0; i<6; i++)
		s << w.w[i] << " ";
	s << "]\n";
return s;

}// end operator<<(std::ostream & s, Ft_v_vector & w)


//Sibi
// Wyciadgniecie maksymalnego elementu z zadanego wektora
double Ft_v_vector::max_element()
{
	double MAX=0;
	int index;

 	for (index=0; index<6; index++)
	{	if ( fabs(w[index]) > MAX) 	{
			MAX=fabs(w[index]); } }

  return MAX;
}// end Ft_v_vector::max_element()


// ******************************************************************************************
//                                           definicje skladowych klasy Ft_v_tr
// ******************************************************************************************

Ft_v_tr::Ft_v_tr()
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz


	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
		{
			if(i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}

	variant = NOT_SET;


}// end Ft_v_tr()

/*
Ft_v_tr::Ft_v_tr(VARIANT variant_l)
{
	// konstruktor domniemany
	// tworzy macierz jednostkowa

	// i - i-ta kolumna
	// j - j-ty wiersz


	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
		{
			if(i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}

	variant = variant_l;


}// end Ft_v_tr()
*/

Ft_v_tr::Ft_v_tr(const Homog_matrix & p, VARIANT variant_l)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	variant = variant_l;

	base_frame = p;

	set_from_frame (base_frame);

}// end Ft_v_tr::Ft_v_tr(const Homog_matrix & p)

// konstruktor kopiujacy
// jest on uzywany podczas inicjalizacji obiektu w momencie jego tworzenia (np. Homog_matrix B = A;)
Ft_v_tr::Ft_v_tr(const Ft_v_tr &wzor)
{
	variant = wzor.variant;

	base_frame = wzor.base_frame;

	set_from_frame(wzor.base_frame);
};//: Homog_matrix::Homog_matrix(const Homog_matrix &wzor)



void Ft_v_tr::set_from_frame(const Homog_matrix & p)
{
	// macierz tworzona jest zgodnie ze wzorem 5.105 ze strony 196 (5.70, 5.72 str 154 - wydanie angielskie)
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	if (variant == NOT_SET) printf("BLAD Ft_v_tr::set_from_frame - FT_V_VARIANT_NOT_SET\n");

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			matrix_m[i][j] = p.matrix_m[i][j];
			matrix_m[i+3][j+3] = p.matrix_m[i][j];

			switch (variant)
			{
				case FT:
					matrix_m[j][i+3] = 0;
				break;
				case V:
					matrix_m[j+3][i] = 0;
				break;
				default:
				break;
			}
		}

	double Porg[3][3];

	Porg[0][0] = 0;
	Porg[0][1] = -p.matrix_m[2][3];
	Porg[0][2] = p.matrix_m[1][3];

	Porg[1][0] = p.matrix_m[2][3];
	Porg[1][1] = 0;
	Porg[1][2] = -p.matrix_m[0][3];

	Porg[2][0] = -p.matrix_m[1][3];
	Porg[2][1] = p.matrix_m[0][3];
	Porg[2][2] = 0;

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		{
			switch (variant)
			{
				case FT:
					matrix_m[j+3][i] = 0;
					for(int a=0; a<3; a++)
					{
						matrix_m[j+3][i] += Porg[j][a] * p.matrix_m[a][i];
					}
				break;
				case V:
					matrix_m[j][i+3] = 0;
					for(int a=0; a<3; a++)
					{
						matrix_m[j][i+3] += Porg[j][a] * p.matrix_m[a][i];
					}
				break;
				default:
				break;
			}
		}

}// end Ft_v_tr::Ft_v_tr(const Homog_matrix & p)



Ft_v_tr Ft_v_tr::operator* (const Ft_v_tr & m) const
{
// mnozenie macierzy

//	Ft_v_tr zwracana;

	// REMOVED BECAUSE IT IS PROBABLY NOT CORRECT TO MULTIPLY IN SUCH A WAY
	/*
	int i, j;			// i - i-ta kolumna
					// j - j-ty wiersz
	// macierz rotacji
	for(i=0; i<6; i++)
	{
		for(j=0; j<6; j++)
		{
			for(int a=0; a<6; a++)
			{
				if(a == 0)
					zwracana.matrix[i][j] = matrix[a][j] * m.matrix[i][a];
				else
					zwracana.matrix[i][j] += matrix[a][j] * m.matrix[i][a];

			}
		}
	}
*/

Ft_v_tr zwracana(base_frame * m.base_frame, variant) ;

return zwracana;
}// end Homog_matrix::operator* (const Homog_matrix & m) const


Ft_v_vector Ft_v_tr::operator*(const Ft_v_vector & w) const
{
	Ft_v_vector zwracany;
	int i;
	int j;
	// i - i-ta kolumna
	// j - j-ty wiersz

	for(j=0;j<6;j++)
		for(i=0;i<6;i++)
			zwracany.w[j] += matrix_m[j][i] * w.w[i];
//	std::cout << "zwracany " << zwracany <<std::endl;
//	std::cout << "w " << zwracany <<std::endl;
return zwracany;
}// Ft_v_tr::operator*(const Ft_v_vector & w) const


Ft_v_tr & Ft_v_tr::operator=(const Ft_v_tr & wzor)
{
	// operator przypisania
	// parametry macierzy przyjmuja wartosc jak parametry macierzy podanej jako argumet

	if(this == &wzor) return *this;

	// do zamiany na mem_cpy

	memcpy(matrix_m, wzor.matrix_m, sizeof(matrix_m));
	/*
	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
			matrix[i][j] = wzor.matrix[i][j];
*/

	variant = wzor.variant;

	base_frame = wzor.base_frame;

return *this;
}// end Ft_v_tr::operator=(const Ft_v_tr & wzor)


Ft_v_tr Ft_v_tr::operator!() const
{
	// przeksztalcenie odwrotne

	Ft_v_tr zwracana;

	zwracana.variant = variant;
	zwracana.base_frame = !base_frame;

	zwracana.set_from_frame(zwracana.base_frame);

return zwracana;

}// end Ft_v_tr::operator!()


std::ostream&  operator<<(std::ostream & strumien, Ft_v_tr & m)
{
	// operator wypisania
	// przedstawia macierz w przyjaznej dla czlowieka formie

	for(int j=0; j<6; j++)
	{
		for(int i=0; i<6; i++)
		{
			strumien << m.matrix_m[j][i] << "\t\t";
		}
		strumien << std::endl;
	}

return strumien;
}// end operator<<(std::ostream & strumien, Ft_v_tr & m)

// ******************************************************************************************
//                                           definicje skladowych klasy Jacobian_matrix
//
//Sibi
// ******************************************************************************************

/* ------------------------------------------------------------------------
Konstruktor klasy - zainicjowane wartosciami zerowymi
------------------------------------------------------------------------ */

Jacobian_matrix::Jacobian_matrix()
{
	for (int i=0; i<=5; i++)
		for(int j=0; j<=5; j++)
			matrix[i][j]=0;
}// end Jacobian_matrix::Jacobian_matrix()


/* ------------------------------------------------------------------------
Wyznaczenie transpozycji jakobianu
------------------------------------------------------------------------ */

void Jacobian_matrix::jacobian_transpose()
{
	double tmp[6][6];	//Tymczasowa macierz z wynikami
	int a;
	int b;

	for (a=0; a<=5; a++){
		for (b=0; b<=5; b++){
			tmp[a][b]=matrix[a][b];}}

	for (a=0; a<=5; a++){
		for(b=0; b<=5; b++){
			matrix[a][b]=tmp[b][a];}}
}//end Jacobian_matrix::jacobian_transpose()

/* ------------------------------------------------------------------------
Przedefiniowanie operatora mnozenia dla macierz * wektor
------------------------------------------------------------------------ */

Ft_v_vector Jacobian_matrix::operator*(const Ft_v_vector & w) const
{
	Ft_v_vector zwracany;
	int i;
	int j;
	for(j=0;j<6;j++)
		for(i=0;i<6;i++)
			zwracany[j] += matrix[j][i] * w.w[i];

	return zwracany;
}//end  Jacobian_matrix::operator*(const Ft_v_vector & w) const

void Jacobian_matrix::wypisz()
{
for (int i=0l; i<6; i++)
	printf("% 7.3f, % 7.3f, % 7.3f, % 7.3f, % 7.3f, % 7.3f\n", matrix[i][0], matrix[i][1], matrix[i][2], matrix[i][3], matrix[i][4], matrix[i][5]);
}//end Jacobian_matrix::wypisz()

// przepisanie elementow jakobianu do  tablicy[6][6] podanej jako argument
void Jacobian_matrix::to_table(double tablica[6][6]) const
{
	for(int i=0; i<6;i++)
		for(int j=0; j<6;j++)
			tablica[i][j]=matrix[i][j];

}// end Ft_v_vector::to_table(double tablica[6]) const

//Rozwiazanie ukladu rowanan Ax=y dla zadanych A i y, metoda eliminacji Gaussa
Ft_v_vector Jacobian_matrix::jacobian_inverse_gauss(const Ft_v_vector & dist)
{
//zmienne pomocnicze w eliminacji Gaussa
double L, a;
int p[6], k, d, i, j, s, tmp;
const double eps=1e-10;
double w[6];

Ft_v_vector q;

//Metoda Eliminacji Gaussa - rozklad LU jakobianu
//Wektor permutacji - czesciowy wybor elementu podstawowego
for (int i=0; i<6; i++){
	p[i]=i;
	w[i]=dist.w[i];}

//V=Jq -> J=PLU -> V=PLUq

for (k=0; k<6; k++){
  a=matrix[p[k]][k]; //wybor elementu podstawowego
  s=k;
  for (d=k+1; d<6; d++){
   if (fabs(a) < fabs(matrix[p[d]][k])) {
     a=matrix[p[d]][k]; //wybierany max element kolumny
     s=d;}}
  if (s!=k){  //modyfikacja wektora permutacji
    tmp=p[k];
    p[k]=p[s];
    p[s]=tmp; }
  for(i=k+1; i<6; i++){ //eliminacja Gaussa
    if (fabs(matrix[p[i]][k])>eps){
      L=matrix[p[i]][k]/a;
      for (j=k+1; j<6; j++){
        matrix[p[i]][j]=matrix[p[i]][j]-L*matrix[p[k]][j];}
      w[p[i]]=w[p[i]]-L*w[p[k]];
	   matrix[p[i]][k]=0;}}}

//Rozwiazanie ukladu r�wnan z macierza gorna trojkatna

q.w[5]=(w[p[5]])/matrix[p[5]][5];
q.w[4]=(w[p[4]]-(q.w[5]*matrix[p[4]][5]))/matrix[p[4]][4];
q.w[3]=(w[p[3]]-(q.w[5]*matrix[p[3]][5]+q.w[4]*matrix[p[3]][4]))/matrix[p[3]][3];
q.w[2]=(w[p[2]]-(q.w[5]*matrix[p[2]][5]+q.w[4]*matrix[p[2]][4]+q.w[3]*matrix[p[2]][3]))/matrix[p[2]][2];
q.w[1]=(w[p[1]]-(q.w[5]*matrix[p[1]][5]+q.w[4]*matrix[p[1]][4]+q.w[3]*matrix[p[1]][3]+q.w[2]*matrix[p[1]][2]))/matrix[p[1]][1];
q.w[0]=(w[p[0]]-(q.w[5]*matrix[p[0]][5]+q.w[4]*matrix[p[0]][4]+q.w[3]*matrix[p[0]][3]+q.w[2]*matrix[p[0]][2]+q.w[1]*matrix[p[0]][1]))/matrix[p[0]][0];

return q;

}//end Ft_v_vector Jacobian_matrix::jacobian_inverse_gauss(const Ft_v_vector & dist)

/* ------------------------------------------------------------------------
  Wyznaczenie jakobianu manipulatora dla zadanej aktualnej konfiguracji - wzory analityczne

  Wejscie:
  * local_current_joints - obecne wartosci wspolrzednych wewnetrznych robota (kolejno q0, q1, q2, ...)
  							   zadane w postaci wektora Ft_v_vector

  Wyjscie:
  * jacobian - macierz jakobianowa
 ------------------------------------------------------------------------ */

void Jacobian_matrix::irp6_6dof_equations(const Ft_v_vector & w)
{

	double s1 = sin(w.w[0]);
	double c1 = cos(w.w[0]);
	double s2 = sin(w.w[1]);
	double c2 = cos(w.w[1]);
	double s3 = sin(w.w[2]);
	double c3 = cos(w.w[2]);
	double s4 = sin(w.w[3]);
	double c4 = cos(w.w[3]);
	double s5 = sin(w.w[4]);
	double c5 = cos(w.w[4]);
	//double s6 = sin(w.w[5]);
	//double c6 = cos(w.w[5]);

	double a2 = 0.455;
  	double a3 = 0.67;
  	double d5 = 0.19;
 	//double d6 = 0.095;
  	//double d7 = 0.20;

	//Wyznaczenie wzoru dla jakobianu

matrix[0][0]=0;
matrix[1][0]=0;
matrix[2][0]=1;
matrix[3][0]=-s1*(c4*d5+c3*a3+c2*a2);
matrix[4][0]=c1*(c4*d5+c3*a3+c2*a2);
matrix[5][0]=0;

matrix[0][1]=0;
matrix[1][1]=0;
matrix[2][1]=0;
matrix[3][1]=-c1*a2*s2;
matrix[4][1]=-s1*a2*s2;
matrix[5][1]=-a2*c2;

matrix[0][2]=0;
matrix[1][2]=0;
matrix[2][2]=0;
matrix[3][2]=-c1*a3*s3;
matrix[4][2]=-s1*a3*s3;
matrix[5][2]=-a3*c3;

matrix[0][3]=-s1;
matrix[1][3]=c1;
matrix[2][3]=0;
matrix[3][3]=-c1*s4*d5;
matrix[4][3]=-s1*s4*d5;
matrix[5][3]=-d5*c4;

matrix[0][4]=c1*c4;
matrix[1][4]=s1*c4;
matrix[2][4]=-s4;
matrix[3][4]=0;
matrix[4][4]=0;
matrix[5][4]=0;

matrix[0][5]=c1*s4*s5-s1*c5;
matrix[1][5]=s1*s4*s5+c1*c5;
matrix[2][5]=c4*s5;
matrix[3][5]=0;
matrix[4][5]=0;
matrix[5][5]=0;

}//end Jacobian_matrix::irp6_6dof_equations(const Ft_v_vector & w)

/* ------------------------------------------------------------------------
  Wyliczenie wartosci wyznacznika jakobianu manipulatora irp6 o 6 stopniach
  swobody dla zadanej aktualnej konfiguracji - wzory analityczne.
  (Wzory bez uwzglednienia narzedzia)

  Wejscie:
  * local_current_joints - obecne wartosci wspolrzednych wewnetrznych robota
						(kolejno q0, q1, q2, ...) zadane w postaci wektora Ft_v_vector

  Wyjscie:
  * det - wartosc wyznacznika macierzy jakobianowej
 ------------------------------------------------------------------------ */

double Jacobian_matrix::irp6_6dof_determinant(const Ft_v_vector & w)
{
	//double s1 = sin(w.w[0]);
	//double c1 = cos(w.w[0]);
	double s2 = sin(w.w[1]);
	double c2 = cos(w.w[1]);
	double s3 = sin(w.w[2]);
	double c3 = cos(w.w[2]);
	//double s4 = sin(w.w[3]);
	double c4 = cos(w.w[3]);
	double s5 = sin(w.w[4]);
	//double c5 = cos(w.w[4]);
	//double s6 = sin(w.w[5]);
	//double c6 = cos(w.w[5]);

	double a2 = 0.455;
  	double a3 = 0.67;
  	double d5 = 0.19;
 	//double d6 = 0.095;
  	//double d7 = 0.20;

	double det;

	det=s5*a2*a3*(a2*c2+d5*c4+a3*c3)*(s2*c3-s3*c2);

	return det;

}//end Jacobian_matrix::irp6_6dof_determinant(const Ft_v_vector & w)

/* ------------------------------------------------------------------------
  Wyznaczenie odwrotnosci jakobianu manipulatora irp6 o 6 stopniach swobody dla zadanej
  aktualnej konfiguracji - wzory analityczne. (Wzory bez uwzglednienia narzedzia)

  Wejscie:
  * local_current_joints - obecne wartosci wspolrzednych wewnetrznych robota
						(kolejno q0, q1, q2, ...) zadane w postaci wektora Ft_v_vector

  Wyjscie:
  * jacobian_inverse - odwrotnosc macierzy jakobianowej
 ------------------------------------------------------------------------ */

void Jacobian_matrix::irp6_6dof_inverse_equations(const Ft_v_vector & w)
{

	double s1 = sin(w.w[0]);
	double c1 = cos(w.w[0]);
	double s2 = sin(w.w[1]);
	double c2 = cos(w.w[1]);
	double s3 = sin(w.w[2]);
	double c3 = cos(w.w[2]);
	double s4 = sin(w.w[3]);
	double c4 = cos(w.w[3]);
	double s5 = sin(w.w[4]);
	double c5 = cos(w.w[4]);
	//double s6 = sin(w.w[5]);
	//double c6 = cos(w.w[5]);

	double a2 = 0.455;
  	double a3 = 0.67;
  	double d5 = 0.19;
 	//double d6 = 0.095;
  	//double d7 = 0.20;

	//Wyznaczenie wzoru na odwrotnosc jakobianu

matrix[0][0] = 0.0;
matrix[0][1] = 0.0;
matrix[0][2] = 0.0;
matrix[0][3] = -s1/(a2*c2+d5*c4+a3*c3);
matrix[0][4] = c1/(a2*c2+d5*c4+a3*c3);
matrix[0][5] = 0.0;

matrix[1][0] = (c1*s3*s4*c5*c4-c1*c3*c5+c1*c3*c5*c4*c4+s3*s1*s5*c4-c3*s4*s1*s5)*d5/(s3*c2-s2*c3)/a2/s5;
matrix[1][1] = -(c1*s3*s5*c4-c1*c3*s4*s5-s1*s3*s4*c5*c4+c3*s1*c5-c3*s1*c5*c4*c4)*d5/(s3*c2-s2*c3)/a2/s5;
matrix[1][2] = (s3*c4-c3*s4)*c4*d5*c5/(s3*c2-s2*c3)/a2/s5;
matrix[1][3] = (c1*c4*d5*c3*s5+c1*c3*c3*a3*s5+c1*c2*a2*c3*s5-c3*s1*c4*d5*s4*c5+s1*s3*c4*c4*d5*c5)/(a2*c2+d5*c4+a3*c3)/(s3*c2-s2*c3)/a2/s5;
matrix[1][4] = -(c1*s3*c4*c4*d5*c5-c1*c3*c4*d5*s4*c5-c2*a2*s1*c3*s5-c4*d5*s1*c3*s5-c3*c3*a3*s1*s5)/(a2*c2+d5*c4+a3*c3)/(s3*c2-s2*c3)/a2/s5;
matrix[1][5] = -s3/a2/(s3*c2-s2*c3);

matrix[2][0] = (c1*c2*c5-c1*c2*c5*c4*c4-c1*s2*c4*s4*c5+c2*s4*s1*s5-s2*c4*s1*s5)*d5/(s3*c2-s2*c3)/a3/s5;
matrix[2][1] = -(c1*c2*s4*s5-c1*s2*c4*s5-c2*s1*c5+c2*s1*c5*c4*c4+s1*s2*c4*s4*c5)*d5/(s3*c2-s2*c3)/a3/s5;
matrix[2][2] = (c2*s4-s2*c4)*c4*d5*c5/(s3*c2-s2*c3)/a3/s5;
matrix[2][3] = -(c1*c4*d5*c2*s5+c1*c3*a3*c2*s5+c1*c2*c2*a2*s5-c2*s1*c4*d5*s4*c5+s1*s2*c4*c4*d5*c5)/(a2*c2+d5*c4+a3*c3)/(s3*c2-s2*c3)/a3/s5;
matrix[2][4] = -(-c1*s2*c4*c4*d5*c5+c1*c2*s4*d5*c4*c5+s1*c2*c2*a2*s5+c2*s1*s5*c4*d5+s1*c3*a3*c2*s5)/(a2*c2+d5*c4+a3*c3)/(s3*c2-s2*c3)/a3/s5;
matrix[2][5] = 1/(s3*c2-s2*c3)/a3*s2;

matrix[3][0] = -(s1*s5+s4*c1*c5)/s5;
matrix[3][1] = (c1*s5-s4*s1*c5)/s5;
matrix[3][2] = -c4*c5/s5;
matrix[3][3] = -s1*c4*c5/(a2*c2+d5*c4+a3*c3)/s5;
matrix[3][4] = c1*c4*c5/(a2*c2+d5*c4+a3*c3)/s5;
matrix[3][5] = 0.0;

matrix[4][0] = c1*c4;
matrix[4][1] = s1*c4;
matrix[4][2] = -s4;
matrix[4][3] = -s4*s1/(a2*c2+d5*c4+a3*c3);
matrix[4][4] = c1*s4/(a2*c2+d5*c4+a3*c3);
matrix[4][5] = 0.0;

matrix[5][0] = c1*s4/s5;
matrix[5][1] = s4*s1/s5;
matrix[5][2] = c4/s5;
matrix[5][3] = c4*s1/(a2*c2+d5*c4+a3*c3)/s5;
matrix[5][4] = -c4*c1/(a2*c2+d5*c4+a3*c3)/s5;
matrix[5][5] = 0.0;

}//end Jacobian_matrix::irp6_6dof_inverse_equations(const Ft_v_vector & w)


} // namespace lib
} // namespace mrrocpp

