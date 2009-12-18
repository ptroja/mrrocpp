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
public:
	enum VARIANT { FT, V, NOT_SET};

private:
	double matrix_m[6][6];												// zmienna przechowujaca parametry macierzy
	Homog_matrix base_frame;										// bazowy trojscian z konstruktora
	 VARIANT variant;

public:

	Ft_v_tr ();																	// kostruktor domniemany
//	Ft_v_tr(VARIANT variant_l);
	Ft_v_tr (const Homog_matrix &, VARIANT variant_l);
	Ft_v_tr(const Ft_v_tr &);												// konstruktor kopiujacy

	void set_from_frame(const Homog_matrix & p);		// ustawia na podstawie trojscianu

	// Mnozenie macierzy.
	Ft_v_tr operator* (const Ft_v_tr & m) const;
	Ft_v_tr operator!() const;

	Ft_v_tr & operator = (const Ft_v_tr &);									// operator przypisania
	Ft_v_vector operator*(const Ft_v_vector &) const;					// mnozenie wektora


	friend std::ostream&  operator<<(std::ostream & strumien, Ft_v_tr &);		// operator wypisania

};// end class Ft_v_tr


} // namespace lib
} // namespace mrrocpp

#endif
