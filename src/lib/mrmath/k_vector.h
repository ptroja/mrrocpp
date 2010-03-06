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


#ifndef __K_VECTOR_H
#define __K_VECTOR_H

#include <Eigen/Core>

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca wektor w kartezjaskim ukaladzie odniesienia
class K_vector : public Eigen::Matrix<double, 3, 1>
{
	typedef Eigen::Matrix<double, 3, 1> BaseClass;

public:
	// Copy constructor from any Eigen matrix type
	template<typename OtherDerived>
	K_vector(const Eigen::MatrixBase<OtherDerived>& other)
		: BaseClass(other)
	{}

	// Reuse assignment operators from base class
	using BaseClass::operator=;

	K_vector ();												// konstruktor domniemany: [0, 0, 0]
	K_vector (double t[3]);								// utworzenie wektora na podstawie tablicy
	K_vector (double x, double y, double z);			// utworzenie wektora na podstawie tablicy

	double get_length() const;
	void normalize();

	void to_table(double tablica[3]) const;			// przepisanie zawartosci do tablicy
};// end class vector



} // namespace lib
} // namespace mrrocpp

#endif
