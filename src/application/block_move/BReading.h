/*
 * BReading.h
 *
 *  Created on: 21-11-2011
 *      Author: spiatek
 */

#ifndef BREADING_H_
#define BREADING_H_

#include "application/visual_servoing/Reading.h"

namespace Types {
namespace Mrrocpp_Proxy {

class BReading: public Reading
{
public:
	BReading()
	{
	}

	virtual ~BReading()
	{
	}

	virtual BReading* clone()
	{
		return new BReading(*this);
	}

	bool rpcReceived;

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <Reading>(*this);

		ar & rpcReceived;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types


#endif /* BREADING_H_ */
