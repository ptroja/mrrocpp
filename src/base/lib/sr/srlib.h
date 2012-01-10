/*!
 * @file srlib.h
 * @brief System reporting declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#if !defined(__SRLIB_H)
#define __SRLIB_H

#include <ctime>
#include <string>
#include <stdint.h>

#include <boost/serialization/serialization.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/Sender.h"

#include "base/lib/exception.h"

namespace mrrocpp {
namespace lib {

//! Length of name strings
//! @bug should not be 'static const' in the message class, because it would be send in message.
//! @todo The solution is to make it std::string and use serialization
#define NAME_LENGTH     30

//! Length of text description in a message
//! @bug should not be 'static const' in the message class, because it would be send in message.
//! @todo The solution is to make it std::string and use serialization
static const unsigned int TEXT_LENGTH = 256;

/*!
 * Package sent to SR.
 */
typedef struct sr_package
{
	//! Message timestamp
	//! @todo use boost::posix_time instead
	struct _portable_timeval
	{
		unsigned long tv_sec;
		unsigned int tv_usec;
	} tv;

	//! Sender process type
	process_type_t process_type;

	//! Message type
	error_class_t message_type;

	//! Sender process name
	char process_name[NAME_LENGTH];

	//! Sender process host
	char host_name[NAME_LENGTH];

	//! Text message
	char description[TEXT_LENGTH];
} sr_package_t;

template <class Archive>
void serialize(Archive & ar, sr_package_t & p, const unsigned int version)
{
	ar & p.tv.tv_sec;
	ar & p.tv.tv_usec;
	ar & p.process_type;
	ar & p.message_type;
	ar & p.process_name;
	ar & p.host_name;
	ar & p.description;
}

//! System reporting (SR)
class sr : public boost::noncopyable
{
private:
	//! Send default message package to the SR
	void send_package(sr_package_t & sr_message);

	//! Sender class
	Sender sender;

	//! Process type
	const process_type_t process_type;

	//! Process name
	const std::string process_name;

	//! Cached hostname
	char hostname[128];

protected:
	//! Interpret the status code into a text message
	virtual void interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1 = 0) = 0;

public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr(process_type_t _process_type, const std::string & _process_name, const std::string & sr_channel_name);

	/**
	 * Destructor
	 */
	virtual ~sr();

	//! Send a message to SR
	//! @bug these methods should be overloaded
	void message(error_class_t message_type, uint64_t error_code);

	//! Send a message to SR
	//! @bug these methods should be overloaded
	void message(error_class_t message_type, uint64_t error_code0, uint64_t error_code1);

	//! Send a message to SR
	//! @bug these methods should be overloaded
	void message(error_class_t message_type, uint64_t error_code, const std::string & text);

	//! Send a message to SR
	//! @bug these methods should be overloaded
	void message(const std::string & text);

	//! Send a message to SR
	//! @bug these methods should be overloaded
	void message(error_class_t message_type, const std::string & text);

	/*!
	 * Sends a message to SR containing information about given MRROC++ error.
	 * \author tkornuta
	 * \date 12.05.2011
	 */
	template <error_class_t ercl>
	void message(const mrrocpp::lib::exception::error <ercl> & e_)
	{
		// A message that will be sent to SR.
		sr_package sr_message;

		// Set error type.
		sr_message.message_type = e_.error_class;

		// Copy error description and diagnostics to message description.
		// Retrieve default description.
		const char* const * pdescription = boost::get_error_info <mrrocpp::lib::exception::error_description>(e_);
		// Check whether description is present.
		if (pdescription != 0)
			strcpy(sr_message.description, (*pdescription));
		else
			strcpy(sr_message.description, "Unidentified error");

		// Add diagnostic information.
		//strcat(sr_message.description, "\n");
		//strcat(sr_message.description, e_.what());
		// TODO: Uncomment lines when UI won't crash anymore;)

		// Send message.
		send_package(sr_message);
	}
};

} // namespace lib
} // namespace mrrocpp

#endif
