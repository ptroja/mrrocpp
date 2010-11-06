#ifndef __FT_V_TR_H
#define __FT_V_TR_H

#include <ostream>

#include "base/lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca macierz transformacji odczytow sily do innego ukladu odniesienia

class Ft_v_tr
{
protected:
	double matrix_m[6][6]; // zmienna przechowujaca parametry macierzy

	Homog_matrix base_frame; // bazowy trojscian z konstruktora

public:

	Ft_v_tr(); // kostruktor domniemany
	// destruktur wirtualny
	virtual ~Ft_v_tr()
	{
	}

	virtual void set_from_frame(const Homog_matrix & p) = 0; // ustawia na podstawie trojscianu

	friend std::ostream& operator<<(std::ostream & strumien, Ft_v_tr &); // operator wypisania
};

class Ft_tr : public Ft_v_tr
{
public:
	Ft_tr(); // kostruktor domniemany
	Ft_tr(const Homog_matrix &);
	Ft_tr(const Ft_tr &); // konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p); // ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Ft_tr operator*(const Ft_tr & m) const;
	Ft_tr operator!() const;

	Ft_tr & operator =(const Ft_tr &); // operator przypisania
	Ft_vector operator*(const Ft_vector &) const; // mnozenie wektora
};

class V_tr : public Ft_v_tr
{
public:
	V_tr(); // kostruktor domniemany
	V_tr(const Homog_matrix &);
	V_tr(const V_tr &); // konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p); // ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	V_tr operator*(const V_tr & m) const;
	V_tr operator!() const;

	V_tr & operator =(const V_tr &); // operator przypisania
	Xyz_Angle_Axis_vector operator*(const Xyz_Angle_Axis_vector &) const; // mnozenie wektora
};

} // namespace lib
} // namespace mrrocpp

#endif
