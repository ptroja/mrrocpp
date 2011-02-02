/*
 * Reading.hpp
 *
 */

#ifndef READING_HPP_
#define READING_HPP_

#include "base/lib/xdr/xdr_oarchive.hpp"

//#include "Logger.hpp"

namespace Proxies {

namespace Mrrocpp {

class Reading
{
public:
	Reading()
	{
	}

	virtual ~Reading()
	{
	}

	virtual Reading* clone() = 0;

	virtual void printInfo()
	{
//		LOG(LNOTICE) << "Reading::printInfo()\n";
	}
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
//		LOG(LNOTICE) << "Reading::serialize()\n";
	}

};

}//namespace Mrrocpp
}//namespace Proxies

#endif /* READING_HPP_ */
