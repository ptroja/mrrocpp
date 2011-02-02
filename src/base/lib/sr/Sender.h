/*!
 * @file Sender.h
 * @brief System reporting sender base class - declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef Sender_H_
#define Sender_H_

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace lib {

//! Forward declaration
typedef struct sr_package sr_package_t;

//! Base class for senders of system report messages
class Sender
{
	//! Descriptor of SR communication channel
	messip_channel_t *ch;

protected:
	//! Send default message package to the SR using underlying transport
	//! @param[in] sr_mess package to send
	void Send(const sr_package_t& package);

public:
	/**
	 * Constructor
	 * @param sr_name name of the communication channel
	 */
	Sender(const std::string & sr_name);

	//! Destructor
	~Sender();

	//! Send package to the receiver
	//! @param[in] package package to send
	void send_package(const sr_package_t& package);
};

} // namespace lib
} // namespace mrrocpp

#endif /* Sender_H_ */
