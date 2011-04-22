/*
 * Reading.hpp
 *
 */

#ifndef READING_HPP_
#define READING_HPP_

#include "base/lib/xdr/xdr_oarchive.hpp"

//#include "Logger.hpp"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 * Reading for MRROC++ proxy.
 */
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

	/**
	 * Serialize object to archive.
	 * @param ar
	 */
	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar) = 0;
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* READING_HPP_ */
