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
#include <Eigen/Core>

#include "base/lib/mrmath/mrmath.h"

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

#define log_message_text_buf_size 1024
#define log_message_time_buf_size 10

struct config_message
{
	config_message();
	char header[log_message_text_buf_size];
	char filename_prefix[log_message_text_buf_size];

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		if(strlen(header) >= log_message_text_buf_size){
			throw std::runtime_error("config_message::serialize(): strlen(header) >= log_message_text_buf_size");
		}
		if(strlen(filename_prefix) >= log_message_text_buf_size){
			throw std::runtime_error("config_message::serialize(): strlen(filename_prefix) >= log_message_text_buf_size");
		}

		ar & header;
		ar & filename_prefix;
	}
};

struct log_message
{
	log_message();

	uint32_t number;
	uint32_t seconds;
	uint32_t nanoseconds;

	char text[log_message_text_buf_size];
	uint32_t time_elems;
	struct timespec time_buf[log_message_time_buf_size];

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & number;
		ar & seconds;
		ar & nanoseconds;
		ar & text;
		ar & time_elems;

		if (strlen(text) >= log_message_text_buf_size) {
			throw std::runtime_error("log_message::serialize(): strlen(text) > log_message_text_buf_size");
		}

		if (time_elems > log_message_time_buf_size) {
			throw std::runtime_error("log_message::serialize(): time_elems > log_message_time_buf_size");
		}

		for (int i = 0; i < time_elems; ++i) {
			ar & time_buf[i].tv_nsec;
			ar & time_buf[i].tv_sec;
		}
	}

	virtual void prepare_text();

	void append_Homog_matrix(const mrrocpp::lib::Homog_matrix& hm);

	template <int rows, int cols>
	void append_matrix(const Eigen::Matrix <double, rows, cols>& mat);
};

template <int rows, int cols>
void log_message::append_matrix(const Eigen::Matrix <double, rows, cols>& mat)
{
	char hm_text[300];
	strcpy(hm_text, "");

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			char v[16];
			sprintf(v, "%0.6lf;", mat(i, j));
			strcat(hm_text, v);
		}
	}
	strcat(text, hm_text);
}

} /* namespace logger */
#endif /* LOG_MESSAGE_H_ */
