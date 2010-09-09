/*!
 * @file SimpleSender.h
 * @brief Simple system reporting sender - declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef SIMPLESENDER_H_
#define SIMPLESENDER_H_

#include <string>

#include "base/lib/sr/SenderBase.h"

namespace mrrocpp {
namespace lib {

//! Non-threaded variant of system reporting sender
class SimpleSender : public SenderBase {
public:
	/**
	 * Constructor
	 * @param sr_name name of the communication channel
	 */
	SimpleSender(const std::string & sr_name);

	//! Destructor
	virtual ~SimpleSender();

	//! Interface method to send package
	//! @param[in] sr_mess package to send
	void send_package(const sr_package_t& package);
};

} // namespace lib
} // namespace mrrocpp

#endif /* SIMPLESENDER_H_ */
