/*
 * PBReading.hpp
 *
 *  Created on: 19-11-2010
 *      Author: mateusz
 */

#ifndef PBREADING_HPP_
#define PBREADING_HPP_

#include "Reading.h"
#include "HomogMatrix.h"
#include <sstream>

namespace Processors {

namespace VisualServoPB {

class PBReading: public Proxies::Mrrocpp::Reading
{
public:
	PBReading()
	{
	}

	PBReading(const PBReading& o)
	{
		objectVisible = o.objectVisible;
		objectPosition = o.objectPosition;
	}

	virtual ~PBReading()
	{
	}

	virtual PBReading* clone()
	{
		return new PBReading(*this);
	}

	bool objectVisible;
	Types::HomogMatrix objectPosition;

	virtual void printInfo()
	{
//		LOG(LNOTICE) << "PBReading::printInfo()\n";
		std::stringstream ss;
		if (objectVisible) {
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 4; ++j) {
					ss << objectPosition.elements[i][j] << "  ";
				}

				ss << "\n";
			}
		} else {
			ss << "object not visible\n";
		}

//		LOG(LNOTICE) << "HomogMatrix:\n" << ss.str() << endl;
	}
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <Reading>(*this);
//		LOG(LTRACE) << "PBReading::serialize()\n";
		ar & objectVisible;
		ar & objectPosition;
	}
};

} // namespace VisualServoPB
} // namespace Processors

#endif /* PBREADING_HPP_ */
