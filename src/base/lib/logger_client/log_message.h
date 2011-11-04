/*
 * log_message.h
 *
 *  Created on: 01-09-2011
 *      Author: mateusz
 */

#ifndef LOG_MESSAGE_H_
#define LOG_MESSAGE_H_

#include <string>
#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace logger {

struct log_message_header
{
	uint32_t message_size;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & message_size;
	}
};

#define message_buf_size 256

struct log_message
{
	uint32_t number;
	uint32_t seconds;
	uint32_t nanoseconds;

	char text[message_buf_size];

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & number;
		ar & seconds;
		ar & nanoseconds;
		ar & text;
	}

	virtual void prepare_text();
};

} /* namespace logger */
#endif /* LOG_MESSAGE_H_ */
