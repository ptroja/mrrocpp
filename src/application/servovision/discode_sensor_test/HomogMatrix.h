/*
 * HomogMatrix.hpp
 *
 *  Created on: 19-11-2010
 *      Author: mateusz
 */

#ifndef HOMOGMATRIX_HPP_
#define HOMOGMATRIX_HPP_

namespace Types {

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

} // namespace Types

#endif /* HOMOGMATRIX_HPP_ */
