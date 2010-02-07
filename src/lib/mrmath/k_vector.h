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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace mrrocpp {
namespace lib {

// klasa reprezentujaca wektor w kartezjaskim ukaladzie odniesienia
class K_vector : public boost::numeric::ublas::bounded_vector<double, 3>
{
	typedef boost::numeric::ublas::bounded_vector<double, 3> Base_vector;

public:
	// Construction and assignment from a uBLAS vector expression or copy assignment
	template <class R> K_vector (const boost::numeric::ublas::vector_expression<R>& r) : Base_vector(r)
	{}

	template <class R> void operator=(const boost::numeric::ublas::vector_expression<R>& r)
	{
		Base_vector::operator=(r);
	}

	template <class R> void operator=(const Base_vector& r)
	{
		Base_vector::operator=(r);
	}

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
