/*!
 * @file ThreadedSender.h
 * @brief Threaded system reporting sender - declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef THREADEDSENDER_H_
#define THREADEDSENDER_H_

#include <string>

#include <boost/thread/condition_variable.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "base/lib/sr/SenderBase.h"
#include "base/lib/sr/srlib.h"

namespace mrrocpp {
namespace lib {

//! Threaded variant of system reporting sender
class ThreadedSender : public SenderBase {

	static const unsigned int SR_BUFFER_LENGHT = 10;

	//! Sender thread in threaded variant
	boost::thread thread_id;

	//! Sender routine in threaded variant
	void operator()();

	//! Container for outgoing messages in threaded variant
	boost::circular_buffer <lib::sr_package_t> cb;

	//! Access lock for outgoing messages container in threaded variant
	mutable boost::mutex mtx;

	//! Wait until a new message arrive
	void wait_for_new_msg();

	//! Get a message from the SR send queue
	//! @param[out] new_msg message to get
	void get_one_msg(lib::sr_package_t& new_msg);

	//! Active command condition
	boost::condition_variable cond;

	//! Flag indicating active command to execute
	bool has_command;

	//! Is the send queue empty?
	bool buffer_empty() const;

public:
	/**
	 * Constructor
	 * @param sr_name name of the communication channel
	 */
	ThreadedSender(const std::string & sr_name);

	//! Destructor
	virtual ~ThreadedSender();

	//! Interface method to send package
	//! @param[in] sr_mess package to send
	void send_package(const sr_package_t& package);
};

} // namespace lib
} // namespace mrrocpp

#endif /* THREADEDSENDER_H_ */
