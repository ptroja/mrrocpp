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

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip.h"
#endif

#include "base/lib/typedefs.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/SenderBase.h"

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

/* -------------------------------------------------------------------- */
/* Paczka danych przesylanych do procesu SR                             */
/* -------------------------------------------------------------------- */
typedef struct sr_package
{
#if !defined(USE_MESSIP_SRR)
	//! QNET header
	//! @bug probably this is not needed
	msg_header_t hdr;
#endif
	//! Message timestamp
	struct timespec ts;

	//! Sender process type
	process_type_t process_type;

	//! Message type
	//! @bug this should be error_class_t, serialized properly after dropping of POD messaging
	int16_t message_type;

	//! Sender process name
	char process_name[NAME_LENGTH];

	//! Sender process host
	char host_name[NAME_LENGTH];

	//! Text message
	char description[TEXT_LENGTH];

	//  sr_package();
} /*__attribute__((__packed__))*/ // not packed in case of msg_header_t
sr_package_t;

template<class Archive>
void serialize(Archive & ar, sr_package_t & p, const unsigned int version)
{
    ar & p.ts.tv_sec;
    ar & p.ts.tv_nsec;
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
	//! Size of the array with error codes
	static const size_t ERROR_TAB_SIZE = 2;

	//! One-thread a time access mutex
	boost::mutex srMutex;

	//! Send default message package to the SR
	void send_package(void);

	//! Interpret the status code into a text message
	virtual void interpret() = 0;

	//! Shared pointer to sender class
	boost::shared_ptr<SenderBase> sender;

protected:
	//! Error codes
	uint64_t error_tab[ERROR_TAB_SIZE];

	//! Package for SR
	sr_package_t sr_message;

public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name, bool _multi_thread);

	/**
	 * Destructor
	 */
	virtual ~sr();

	//! Send a message to SR
	//! @bug these methods should me overloaded
	//! @bug this method does not clear error_tab[1]
	void message(error_class_t message_type, uint64_t error_code);

	//! Send a message to SR
	//! @bug these methods should me overloaded
	void message(error_class_t message_type, uint64_t error_code0, uint64_t error_code1);

	//! Send a message to SR
	//! @bug these methods should me overloaded
	void message(error_class_t message_type, uint64_t error_code, const std::string & text);

	//! Send a message to SR
	//! @bug these methods should me overloaded
	void message(const std::string & text);

	//! Send a message to SR
	//! @bug these methods should me overloaded
	void message(error_class_t message_type, const std::string & text);
};

} // namespace lib
} // namespace mrrocpp

#endif
