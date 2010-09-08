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

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#if defined(USE_MESSIP_SRR)
#include "messip.h"
#endif

#include "base/lib/typedefs.h"
#include "base/lib/com_buf.h"

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

//! System reporting (SR)
class sr : public boost::noncopyable
{
private:
	// Rozmiar tablicy zawierajacej kody bledow
	static const unsigned int ERROR_TAB_SIZE = 2;

	static const unsigned int SR_BUFFER_LENGHT = 10;

	//! Active command condition
	boost::condition_variable cond;

	//! Flag indicating active command to execute
	bool has_command;

	//! Put a message to the SR send queue
	//! @param[in] new_msg message to put
	void put_one_msg(const lib::sr_package_t& new_msg);

	//! Get a message from the SR send queue
	//! @param[out] new_msg message to get
	void get_one_msg(lib::sr_package_t& new_msg);

	//! Is the send queue empty?
	bool buffer_empty() const;

	//! One-thread a time access mutex
	boost::mutex srMutex;

	//! Wait until a new message arrive
	void wait_for_new_msg();

	//! Send default message package to the SR
	void send_package(void);

	//! Send default message package to the SR
	//! @param[in] sr_mess package to send
	void send_package_to_sr(const sr_package_t& package);

	//! If we are running a multi-threaded variant of this class
	//! @bug This should be done as an inheritance or policy
	const bool multi_thread;

	//! Sender thread in multi-threaded variant
	//! @bug This should be done as an inheritance or policy
	boost::thread *thread_id;

	//! Sender routine in multi-threaded variant
	//! @bug This should be done as an inheritance or policy
	void operator()();

	//! Container for outgoing messages in multi-threaded variant
	//! @bug This should be done as an inheritance or policy
	boost::circular_buffer <lib::sr_package_t> cb;

	//! Access lock for outgoing messages conatiner in multi-threaded variant
	//! @bug This should be done as an inheritance or policy
	mutable boost::mutex mtx;

	//! Interpret the status code into a text message
	virtual void interpret() = 0;

#if !defined(USE_MESSIP_SRR)
	//! Descriptor of SR communication channel
	int fd;
#else
	//! Descriptor of SR communication channel
	messip_channel_t *ch;
#endif /* !USE_MESSIP_SRR */

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

//! SR class for use in EDP
class sr_edp : public sr
{
public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr_edp(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name, bool _multi_thread =	false);
protected:
	virtual void interpret(void);
};

//! SR class for use in ECP
class sr_ecp : public sr
{
public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr_ecp(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name, bool _multi_thread =	false);

protected:
	//! Interpret the status code into a text message
	virtual void interpret(void);
};

//! SR class for use in VSP
class sr_vsp : public sr
{
public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr_vsp(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name, bool _multi_thread = false);

protected:
	//! Interpret the status code into a text message
	virtual void interpret(void);
};

//! SR class for use in UI
class sr_ui : public sr
{
public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr_ui(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name, bool _multi_thread = false);

protected:
	//! Interpret the status code into a text message
	virtual void interpret(void);
};

} // namespace lib
} // namespace mrrocpp

#endif
