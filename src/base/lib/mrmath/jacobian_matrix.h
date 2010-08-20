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

#ifndef __JACOBIAN_MATRIX_H
#define __JACOBIAN_MATRIX_H

#include "base/lib/impconst.h"	// frame_tab

namespace mrrocpp {
namespace lib {

//Sibi
// klasa reprezentujaca macierz jakobianu 6 na 6
class Jacobian_matrix
{
private:
	double matrix[6][6];												// Miejsce na jakobian

public:
	Jacobian_matrix ();													// kostruktor domniemany

     void irp6_6dof_equations(const Xyz_Angle_Axis_vector & w);			//Wzory na jakobian dla Irp-6 o 6 stopniach swobody
     void irp6_6dof_inverse_equations(const Xyz_Angle_Axis_vector & w);				//Wzory na odwrotnosc jakobianu dla Irp-6 o 6 stopniach swobody
	 double irp6_6dof_determinant(const Xyz_Angle_Axis_vector & w);					//Wzory na wyznacznik jaokbianu dla Irp-6 o 6 stopniach swobody

     void jacobian_transpose();										//Wyznaczenie transpozycji jakobianu
     void wypisz();														//Wypisanie zawartosci macierzy na konsole
     void to_table(double tablica[6][6]) const;						// przepisanie elementw jakobianu do tablicy[6][6] podanej jako argument

     Xyz_Angle_Axis_vector jacobian_inverse_gauss(const Xyz_Angle_Axis_vector & dist);				//Rozwiazanie ukladu rownan AX=Y (A, Y - zadane)
																								//za pomoca metody eliminacji Gaussa
     Xyz_Angle_Axis_vector operator* (const Xyz_Angle_Axis_vector & w) const;		//Przeciazenie operacji mnozenia dla jakobianu i wektora

};// end class Jacobian_matrix

} // namespace lib
} // namespace mrrocpp

#endif
