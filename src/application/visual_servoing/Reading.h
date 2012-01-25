/*
 * Reading.hpp
 *
 */

#ifndef READING_HPP_
#define READING_HPP_

#include <iostream>

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
	Reading() :
		processingStartSeconds(0), processingStartNanoseconds(0), processingEndSeconds(0), processingEndNanoseconds(0)
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
	virtual void send(boost::shared_ptr <xdr_oarchive <> > & ar) = 0;

	/**
	 * Timestamp when processing starts (taken just after camera source).
	 */
	uint32_t processingStartSeconds;
	uint32_t processingStartNanoseconds;

	/**
	 * Timestamp when processing ends (taken just before sending to mrroc proxy).
	 */
	uint32_t processingEndSeconds;
	uint32_t processingEndNanoseconds;
private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		//		std::cout << "Reading::serialize()\n";

		ar & processingStartSeconds;
		ar & processingStartNanoseconds;

		ar & processingEndSeconds;
		ar & processingEndNanoseconds;

		//		std::cout << "processingStartSeconds = " << processingStartSeconds << "\n";
		//		std::cout << "processingStartNanoseconds = " << processingStartNanoseconds<< "\n";
		//		std::cout << "processingEndSeconds = " << processingEndSeconds<< "\n";
		//		std::cout << "processingEndNanoseconds = " << processingEndNanoseconds<< "\n";
		//		std::cout << "\n";
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* READING_HPP_ */
