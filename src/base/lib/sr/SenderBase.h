/*!
 * @file SenderBase.h
 * @brief System reporting sender base class - declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef SENDERBASE_H_
#define SENDERBASE_H_

#include <string>


#include "base/lib/messip/messip_dataport.h"


namespace mrrocpp {
namespace lib {

//! Forward declaration
typedef struct sr_package sr_package_t;

//! Base class for senders of system report messages
class SenderBase
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
	SenderBase(const std::string & sr_name);

	//! Destructor
	virtual ~SenderBase();

	//! Abstract interface method
	//! @param[in] sr_mess package to send
	virtual void send_package(const sr_package_t& package) = 0;
};

} // namespace lib
} // namespace mrrocpp

#endif /* SENDERBASE_H_ */
