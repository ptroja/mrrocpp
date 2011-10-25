/*
 * HomogMatrix.hpp
 *
 *  Created on: 19-11-2010
 *      Author: mateusz
 */

#ifndef HOMOGMATRIX_HPP_
#define HOMOGMATRIX_HPP_

#include "base/lib/mrmath/homog_matrix.h"

namespace Types {

// Note: this assumes, that the DisCODe type serialization conforms with the MRROC++ type
typedef lib::Homog_matrix HomogMatrix;

#if 0
struct HomogMatrix
{
	double elements[3][4];
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & elements;
	}
};
#endif

} // namespace Types

#endif /* HOMOGMATRIX_HPP_ */
