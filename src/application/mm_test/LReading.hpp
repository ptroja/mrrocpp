

#ifndef LREADING_HPP_
#define LREADING_HPP_

#include "application/visual_servoing/Reading.h"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 *
 */
class LReading: public Reading
{
public:
	LReading()
	{
	}

	LReading(const LReading& o)
	{
		info = o.info;
	}

	virtual ~LReading()
	{
	}

	virtual LReading* clone()
	{
		return new LReading(*this);
	}

	double info;

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <Reading>(*this);
		//LOG(LTRACE) << "LReading::serialize()\n";

		ar & info;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* LREADING_HPP_ */
