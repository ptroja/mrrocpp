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

#ifndef __MATHTR_H
#define __MATHTR_H

namespace mrrocpp {
namespace lib {

// Sprowadzenie wartosci kata do przedzialu <-pi,pi>
double reduce(double angle);
double reduce(double angle, double min, double max, double offset);

} // namespace lib
} // namespace mrrocpp

#endif
