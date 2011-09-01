/*
 * log_message.h
 *
 *  Created on: 01-09-2011
 *      Author: mateusz
 */

#ifndef LOG_MESSAGE_H_
#define LOG_MESSAGE_H_

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace logger {

struct log_message
{
	uint32_t number;
	uint32_t seconds;
	uint32_t nanoseconds;

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & number;
		ar & seconds;
		ar & nanoseconds;
	}

	virtual log_message* clone() const = 0;
};

} /* namespace logger */
#endif /* LOG_MESSAGE_H_ */
