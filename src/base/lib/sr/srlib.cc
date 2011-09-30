/*!
 * @file srlib.cc
 * @brief System reporting.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <cstdio>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "base/lib/sr/Sender.h"

namespace mrrocpp {
namespace lib {

sr::sr(process_type_t _process_type, const std::string & _process_name, const std::string & sr_name)
	: sender(sr_name), process_type(_process_type), process_name(_process_name)
{
	if(gethostname(hostname, sizeof(hostname)) == -1) {
		perror("gethostname()");
		hostname[0] = '\0';
	}
}

void sr::send_package(sr_package_t & sr_message)
{
	sr_message.process_type = process_type;
	strncpy(sr_message.process_name, process_name.c_str(), sizeof(sr_message.process_name));
	strncpy(sr_message.host_name, hostname, sizeof(sr_message.host_name));

	struct timeval tv;
	if(gettimeofday(&tv, NULL) == -1) {
		perror("gettimeofday()");
	}

	sr_message.tv.tv_sec = tv.tv_sec;
	sr_message.tv.tv_usec = tv.tv_usec;

	sender.send_package(sr_message);
}

sr::~sr()
{
}

void sr::message(const std::string & text)
{
	message(NEW_MESSAGE, text);
}

void sr::message(error_class_t message_type, uint64_t error_code)
{
	message(message_type, error_code, 0);
}

void sr::message(error_class_t message_type, const std::string & text)
{
	sr_package sr_message;

	sr_message.message_type = message_type;

	if (text.length()) {
		strncpy(sr_message.description, text.c_str(), TEXT_LENGTH);
		sr_message.description[TEXT_LENGTH - 1] = '\0';
	} else {
		sr_message.description[0] = '\0';
	}

	return send_package(sr_message);
}

void sr::message(error_class_t message_type, uint64_t error_code, const std::string & text)
{
	sr_package sr_message;

	sr_message.message_type = message_type;

	interpret(sr_message.description, message_type, error_code);

	strcat(sr_message.description, text.c_str());

	return send_package(sr_message);
}

void sr::message(error_class_t message_type, uint64_t error_code0, uint64_t error_code1)
{
	sr_package sr_message;

	sr_message.message_type = message_type;

	interpret(sr_message.description, message_type, error_code0, error_code1);

	send_package(sr_message);
}



} // namespace lib
} // namespace mrrocpp
