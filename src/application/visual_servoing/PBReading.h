/*
 * PBReading.hpp
 *
 *  Created on: 19-11-2010
 *      Author: mateusz
 */

#ifndef PBREADING_HPP_
#define PBREADING_HPP_

#include <sstream>

#include "Reading.h"
#include "HomogMatrix.h"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 *
 */
class PBReading: public Reading
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

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
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

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* PBREADING_HPP_ */
