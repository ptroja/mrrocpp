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


/////////////////////////////////////////
//
//  Ft_vector
//
//////////////////////////////////////////



Ft_vector::Ft_vector() : Ft_v_vector()
{}

Ft_vector::Ft_vector(const double t[6]) : Ft_v_vector(t)
{}

Ft_vector::Ft_vector(double fx, double fy, double fz, double tx, double ty, double tz)
: Ft_v_vector(fx, fy, fz, tx, ty, tz)
{}

Ft_vector::Ft_vector(const Ft_vector & wzor) : Ft_v_vector(wzor)
{}

Ft_vector & Ft_vector::operator=(const Ft_vector & p)
{
	// operator przypisania
	// parametry wektora przyjmuja wartosci takie jak parametry wektora podanego jako argument

	if(this == &p) return *this;

	set_values(p.w);

return *this;
}// end Ft_v_vector::operator=(const Ft_v_vector & p)

Ft_vector Ft_vector::operator+(const Ft_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Ft_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] + dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const


Ft_vector Ft_vector::operator-(const Ft_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Ft_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] - dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const



Ft_vector Ft_vector::operator*(const double skalar) const
{
	Ft_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}


Ft_vector Ft_vector::operator!() const
{
	// odwrocenie wektora

	Ft_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


Ft_vector Ft_vector::operator-() const
{
	// odwrocenie wektora

	Ft_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


/////////////////////////////////////////
//
//  Xyz_Angle_Axis_vector
//
//////////////////////////////////////////




Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector() : Ft_v_vector()
{}

Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector(const double t[6]) : Ft_v_vector(t)
{}

Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector(double fx, double fy, double fz, double tx, double ty, double tz)
: Ft_v_vector(fx, fy, fz, tx, ty, tz)
{}

Xyz_Angle_Axis_vector::Xyz_Angle_Axis_vector(const Xyz_Angle_Axis_vector & wzor) : Ft_v_vector(wzor)
{}

Xyz_Angle_Axis_vector & Xyz_Angle_Axis_vector::operator=(const Xyz_Angle_Axis_vector & p)
{
	// operator przypisania
	// parametry wektora przyjmuja wartosci takie jak parametry wektora podanego jako argument

	if(this == &p) return *this;

	set_values(p.w);

return *this;
}// end Ft_v_vector::operator=(const Ft_v_vector & p)

Xyz_Angle_Axis_vector Xyz_Angle_Axis_vector::operator+(const Xyz_Angle_Axis_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Xyz_Angle_Axis_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] + dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const


Xyz_Angle_Axis_vector Xyz_Angle_Axis_vector::operator-(const Xyz_Angle_Axis_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Xyz_Angle_Axis_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] - dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const



Xyz_Angle_Axis_vector Xyz_Angle_Axis_vector::operator*(const double skalar) const
{
	Xyz_Angle_Axis_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}


Xyz_Angle_Axis_vector Xyz_Angle_Axis_vector::operator!() const
{
	// odwrocenie wektora

	Xyz_Angle_Axis_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


Xyz_Angle_Axis_vector Xyz_Angle_Axis_vector::operator-() const
{
	// odwrocenie wektora

	Xyz_Angle_Axis_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}




void Xyz_Angle_Axis_vector::position_distance(const Homog_matrix& local_current_end_effector_frame, const Homog_matrix& local_desired_end_effector_frame)
{

double n_t[3], n_d[3], o_t[3], o_d[3], a_t[3], a_d[3];

//Wyliczenie wektora przesuniecia : w - predkosc katowa
// n, o, a - wektory normalny, orientacji i zbli�enia

n_t[0]=(local_current_end_effector_frame)[0][0];
n_t[1]=(local_current_end_effector_frame)[1][0];
n_t[2]=(local_current_end_effector_frame)[2][0];

n_d[0]=(local_desired_end_effector_frame)[0][0];
n_d[1]=(local_desired_end_effector_frame)[1][0];
n_d[2]=(local_desired_end_effector_frame)[2][0];

o_t[0]=(local_current_end_effector_frame)[0][1];
o_t[1]=(local_current_end_effector_frame)[1][1];
o_t[2]=(local_current_end_effector_frame)[2][1];

o_d[0]=(local_desired_end_effector_frame)[0][1];
o_d[1]=(local_desired_end_effector_frame)[1][1];
o_d[2]=(local_desired_end_effector_frame)[2][1];

a_t[0]=(local_current_end_effector_frame)[0][2];
a_t[1]=(local_current_end_effector_frame)[1][2];
a_t[2]=(local_current_end_effector_frame)[2][2];

a_d[0]=(local_desired_end_effector_frame)[0][2];
a_d[1]=(local_desired_end_effector_frame)[1][2];
a_d[2]=(local_desired_end_effector_frame)[2][2];

//Wyliczenie wektora przesuniecia : v - predkosc obrotowa

w[0]=(0.5)*((n_t[1]*n_d[2]-n_t[2]*n_d[1])+(o_t[1]*o_d[2]-o_t[2]*o_d[1])+(a_t[1]*a_d[2]-a_t[2]*a_d[1]));
w[1]=(0.5)*((n_t[2]*n_d[0]-n_t[0]*n_d[2])+(o_t[2]*o_d[0]-o_t[0]*o_d[2])+(a_t[2]*a_d[0]-a_t[0]*a_d[2]));
w[2]=(0.5)*((n_t[0]*n_d[1]-n_t[1]*n_d[0])+(o_t[0]*o_d[1]-o_t[1]*o_d[0])+(a_t[0]*a_d[1]-a_t[1]*a_d[0]));

//Wyliczenie wektora przesuniecia : v - predkosc liniowa

w[3]=(local_desired_end_effector_frame)[0][3]-(local_current_end_effector_frame)[0][3];
w[4]=(local_desired_end_effector_frame)[1][3]-(local_current_end_effector_frame)[1][3];
w[5]=(local_desired_end_effector_frame)[2][3]-(local_current_end_effector_frame)[2][3];
}// end V_vector::position_distance(frame_tab*, frame_tab* )



/////////////////////////////////////////
//
//  Xyz_Euler_Zyz_vector
//
//////////////////////////////////////////




Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector() : Ft_v_vector()
{}

Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector(const double t[6]) : Ft_v_vector(t)
{}

Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector(double fx, double fy, double fz, double tx, double ty, double tz)
: Ft_v_vector(fx, fy, fz, tx, ty, tz)
{}

Xyz_Euler_Zyz_vector::Xyz_Euler_Zyz_vector(const Xyz_Euler_Zyz_vector & wzor) : Ft_v_vector(wzor)
{}

Xyz_Euler_Zyz_vector & Xyz_Euler_Zyz_vector::operator=(const Xyz_Euler_Zyz_vector & p)
{
	// operator przypisania
	// parametry wektora przyjmuja wartosci takie jak parametry wektora podanego jako argument

	if(this == &p) return *this;

	set_values(p.w);

return *this;
}// end Ft_v_vector::operator=(const Ft_v_vector & p)

Xyz_Euler_Zyz_vector Xyz_Euler_Zyz_vector::operator+(const Xyz_Euler_Zyz_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Xyz_Euler_Zyz_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] + dod.w[i];

return zwracany;

}// end Xyz_Euler_Zyz_vector::operator+(const Xyz_Euler_Zyz_vector & dod) const


Xyz_Euler_Zyz_vector Xyz_Euler_Zyz_vector::operator-(const Xyz_Euler_Zyz_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Xyz_Euler_Zyz_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] - dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const



Xyz_Euler_Zyz_vector Xyz_Euler_Zyz_vector::operator*(const double skalar) const
{
	Xyz_Euler_Zyz_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}


Xyz_Euler_Zyz_vector Xyz_Euler_Zyz_vector::operator!() const
{
	// odwrocenie wektora

	Xyz_Euler_Zyz_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


Xyz_Euler_Zyz_vector Xyz_Euler_Zyz_vector::operator-() const
{
	// odwrocenie wektora

	Xyz_Euler_Zyz_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}



/////////////////////////////////////////
//
//  Xyz_Rpy_vector
//
//////////////////////////////////////////




Xyz_Rpy_vector::Xyz_Rpy_vector() : Ft_v_vector()
{}

Xyz_Rpy_vector::Xyz_Rpy_vector(const double t[6]) : Ft_v_vector(t)
{}

Xyz_Rpy_vector::Xyz_Rpy_vector(double fx, double fy, double fz, double tx, double ty, double tz)
: Ft_v_vector(fx, fy, fz, tx, ty, tz)
{}

Xyz_Rpy_vector::Xyz_Rpy_vector(const Xyz_Rpy_vector & wzor) : Ft_v_vector(wzor)
{}

Xyz_Rpy_vector & Xyz_Rpy_vector::operator=(const Xyz_Rpy_vector & p)
{
	// operator przypisania
	// parametry wektora przyjmuja wartosci takie jak parametry wektora podanego jako argument

	if(this == &p) return *this;

	set_values(p.w);

return *this;
}// end Ft_v_vector::operator=(const Ft_v_vector & p)

Xyz_Rpy_vector Xyz_Rpy_vector::operator+(const Xyz_Rpy_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Xyz_Rpy_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] + dod.w[i];

return zwracany;

}// end Xyz_Euler_Zyz_vector::operator+(const Xyz_Euler_Zyz_vector & dod) const


Xyz_Rpy_vector Xyz_Rpy_vector::operator-(const Xyz_Rpy_vector & dod) const
{
	// operator dodawania
	// umozliwia zmodyfikowanie parametrow wersora zgodnie z zawartoscia argumentu

	Xyz_Rpy_vector zwracany;

	for(int i=0; i<6; i++)
		zwracany.w[i] = w[i] - dod.w[i];

return zwracany;

}// end Ft_v_vector::operator+(const Ft_v_vector & dod) const



Xyz_Rpy_vector Xyz_Rpy_vector::operator*(const double skalar) const
{
	Xyz_Rpy_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=w[i]*skalar;
	return zwracany;
}


Xyz_Rpy_vector Xyz_Rpy_vector::operator!() const
{
	// odwrocenie wektora

	Xyz_Rpy_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}


Xyz_Rpy_vector Xyz_Rpy_vector::operator-() const
{
	// odwrocenie wektora

	Xyz_Rpy_vector zwracany;
	for(int i=0;i<6;i++) zwracany.w[i]=-w[i];
	return zwracany;
}




} // namespace lib
} // namespace mrrocpp

