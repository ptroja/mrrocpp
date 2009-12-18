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


} // namespace lib
} // namespace mrrocpp

