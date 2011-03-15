/*
 * ImagePosition.hpp
 *
 *  Created on: 30-11-2010
 *      Author: mboryn
 */

#ifndef IMAGEPOSITION_HPP_
#define IMAGEPOSITION_HPP_

#include <boost/serialization/serialization.hpp>
#include <iostream>

namespace Types {

/**
 * Object's position in image.
 * Contains image coordinates.
 */
struct ImagePosition {
	static const int elementsSize = 4;
	double elements[elementsSize];
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & elements;
	}

	friend std::ostream& operator<<(std::ostream& os, const ImagePosition& ip);
};

std::ostream& operator<<(std::ostream& os, const ImagePosition& ip)
{
	os<<"[";
	for(int i=0; i<ImagePosition::elementsSize; ++i){
		os<<ip.elements[i];
		if(i < ImagePosition::elementsSize-1){
			os<<"; ";
		}
	}
	os<<"]";

	return os;
}

} // namespace Types

#endif /* IMAGEPOSITION_HPP_ */
