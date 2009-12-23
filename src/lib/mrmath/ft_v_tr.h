// ------------------------------------------------------------------------
// Proces:		-
// Plik:			mathtr.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Klasy K_vector, Homog_matrix, Ft_v_vector,  Jacobian_matrix
//				- definicja klas
//
// Autor:		tkornuta
// Data:		14.02.2007
// ------------------------------------------------------------------------


#ifndef __FT_V_TR_H
#define __FT_V_TR_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca macierz transformacji odczytow sily do innego ukladu odniesienia

class Ft_v_tr
{

protected:
	double matrix_m[6][6];												// zmienna przechowujaca parametry macierzy
	Homog_matrix base_frame;										// bazowy trojscian z konstruktora

public:

	Ft_v_tr ();																	// kostruktor domniemany

	virtual void set_from_frame(const Homog_matrix & p) = 0;		// ustawia na podstawie trojscianu

	friend std::ostream&  operator<<(std::ostream & strumien, Ft_v_tr &);		// operator wypisania

};// end class Ft_v_tr



class Ft_tr : public Ft_v_tr
{

public:

	Ft_tr ();																	// kostruktor domniemany
//	Ft_v_tr(VARIANT variant_l);
	Ft_tr (const Homog_matrix &);
	Ft_tr(const Ft_tr &);												// konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p);		// ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Ft_tr operator* (const Ft_tr & m) const;
	Ft_tr operator!() const;

	Ft_tr & operator = (const Ft_tr &);									// operator przypisania
	Ft_vector operator*(const Ft_vector &) const;					// mnozenie wektora


};// end class Ft_v_tr


class V_tr : public Ft_v_tr
{

public:

	V_tr ();																	// kostruktor domniemany
//	Ft_v_tr(VARIANT variant_l);
	V_tr (const Homog_matrix &);
	V_tr(const V_tr &);												// konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p);		// ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	V_tr operator* (const V_tr & m) const;
	V_tr operator!() const;

	V_tr & operator = (const V_tr &);									// operator przypisania
	V_vector operator*(const V_vector &) const;					// mnozenie wektora


};// end class Ft_v_tr




} // namespace lib
} // namespace mrrocpp

#endif
