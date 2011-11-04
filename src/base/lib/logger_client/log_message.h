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
#include <time.h>
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

#define log_message_text_buf_size 256
#define log_message_timespec_buf_size 10

struct log_message
{
	log_message();

	uint32_t number;
	uint32_t seconds;
	uint32_t nanoseconds;

	char text[log_message_text_buf_size];
	uint32_t time_elems;
	struct timespec time_buf[log_message_timespec_buf_size];

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		//		ar & number;
		//		ar & seconds;
		//		ar & nanoseconds;
		//		ar & text;

		ar & number;
		ar & seconds;
		ar & nanoseconds;
		ar & text;
		ar & time_elems;
		for (int i = 0; i < time_elems; ++i) {
			ar & time_buf[i].tv_nsec;
			ar & time_buf[i].tv_sec;
		}
	}

	//	template <class Archive>
	//	void save(Archive & ar, const unsigned int version) const
	//	{
	//
	//	}
	//	template <class Archive>
	//	void load(Archive & ar, const unsigned int version)
	//	{
	//		ar & number;
	//		ar & seconds;
	//		ar & nanoseconds;
	//		ar & text;
	//	}

	//	BOOST_SERIALIZATION_SPLIT_MEMBER()

	virtual void prepare_text();
};

} /* namespace logger */
#endif /* LOG_MESSAGE_H_ */
