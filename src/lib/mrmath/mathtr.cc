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

// Zwrocenie dlugosci wektora.
double K_vector::get_length() const
{
	return sqrt (w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
}


// Zwrocenie dligosci wektora.
void K_vector::normalize()
{
	const double length = get_length();
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
double K_vector::operator[](const int i) const
{
	//    printf("RHS a[%2d]\n", i );
	return w[i];
}

// in theory, the LHS operator
double& K_vector::operator[](const int i)
{
	// printf("LHS a[%2d]\n", i );
	return w[i];
}



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
  double Ft_v_vector::operator[](const int i ) const
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

 	for (int index=0; index<6; index++)
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
}

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
for (int i=0; i<6; i++)
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

	const double s1 = sin(w.w[0]);
	const double c1 = cos(w.w[0]);
	const double s2 = sin(w.w[1]);
	const double c2 = cos(w.w[1]);
	const double s3 = sin(w.w[2]);
	const double c3 = cos(w.w[2]);
	const double s4 = sin(w.w[3]);
	const double c4 = cos(w.w[3]);
	const double s5 = sin(w.w[4]);
	const double c5 = cos(w.w[4]);
	//const double s6 = sin(w.w[5]);
	//const double c6 = cos(w.w[5]);

	const double a2 = 0.455;
	const double a3 = 0.67;
	const double d5 = 0.19;
 	//const double d6 = 0.095;
  	//const double d7 = 0.20;

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
	//const double s1 = sin(w.w[0]);
	//const double c1 = cos(w.w[0]);
	const double s2 = sin(w.w[1]);
	const double c2 = cos(w.w[1]);
	const double s3 = sin(w.w[2]);
	const double c3 = cos(w.w[2]);
	//const double s4 = sin(w.w[3]);
	const double c4 = cos(w.w[3]);
	const double s5 = sin(w.w[4]);
	//const double c5 = cos(w.w[4]);
	//const double s6 = sin(w.w[5]);
	//const double c6 = cos(w.w[5]);

	const double a2 = 0.455;
	const double a3 = 0.67;
  	const double d5 = 0.19;
 	//const double d6 = 0.095;
  	//const double d7 = 0.20;

  	const double det = s5*a2*a3*(a2*c2+d5*c4+a3*c3)*(s2*c3-s3*c2);

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
	const double s1 = sin(w.w[0]);
	const double c1 = cos(w.w[0]);
	const double s2 = sin(w.w[1]);
	const double c2 = cos(w.w[1]);
	const double s3 = sin(w.w[2]);
	const double c3 = cos(w.w[2]);
	const double s4 = sin(w.w[3]);
	const double c4 = cos(w.w[3]);
	const double s5 = sin(w.w[4]);
	const double c5 = cos(w.w[4]);
	//const double s6 = sin(w.w[5]);
	//const double c6 = cos(w.w[5]);

	const double a2 = 0.455;
	const double a3 = 0.67;
	const double d5 = 0.19;
 	//const double d6 = 0.095;
  	//const double d7 = 0.20;

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

