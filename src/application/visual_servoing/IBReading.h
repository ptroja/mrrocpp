/*
 * PBReading.hpp
 *
 *  Created on: 19-11-2010
 *      Author: mateusz
 */

#ifndef IBREADING_HPP_
#define IBREADING_HPP_

#include "Reading.h"
#include "ImagePosition.h"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 *
 */
class IBReading: public Reading
{
public:
	IBReading()
	{
	}

	virtual ~IBReading()
	{
	}

	virtual IBReading* clone()
	{
		return new IBReading(*this);
	}

	bool objectVisible;
	Types::ImagePosition imagePosition;

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
//		LOG(LTRACE) << "IBReading::serialize()\n";
		ar & boost::serialization::base_object <Reading>(*this);

		ar & objectVisible;
		ar & imagePosition;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* IBREADING_HPP_ */
