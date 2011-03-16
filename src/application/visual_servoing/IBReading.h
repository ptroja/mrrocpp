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

	IBReading(const IBReading& o)
	{
		objectVisible = o.objectVisible;
		imagePosition = o.imagePosition;
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
//		LOG(LNOTICE) << "IBReading::send(): hehehehe\n";
		*ar<<*this;
	}
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <Reading>(*this);
//		LOG(LTRACE) << "IBReading::serialize()\n";
		ar & objectVisible;
		ar & imagePosition;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* IBREADING_HPP_ */
