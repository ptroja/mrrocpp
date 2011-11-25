/*
 * log_writer.h
 *
 *  Created on: 03-09-2011
 *      Author: mateusz
 */

#ifndef LOG_WRITER_H_
#define LOG_WRITER_H_

#include <string>

#include "base/lib/logger/log_message.h"

namespace logger {

class log_writer
{
public:
	log_writer();
	virtual ~log_writer();

	void close_all();

	void write_msg(std::string connection_name, std::string client_name, const log_message* msg);
protected:

private:

};

} /* namespace logger */
#endif /* LOG_WRITER_H_ */
