/*!
 * @file sr_ecp.cc
 * @brief System reporter class for ECP - definitions.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef SR_ECP_H_
#define SR_ECP_H_

#include <string>

#include "base/lib/sr/srlib.h"

namespace mrrocpp {
namespace lib {

//! SR class for use in ECP
class sr_ecp : public sr
{
protected:
	//! Interpret the status code into a text message
	virtual void interpret(char * description, error_class_t message_type, uint64_t error_code0, uint64_t error_code1);

public:
	/**
	 * Constructor
	 * @param process_type reporting process type
	 * @param process_name reporting process name
	 * @param sr_channel_name name of the SR communication channel
	 * @param _multi_thread flag for selecting multi-threaded variant
	 */
	sr_ecp(process_type_t process_type, const std::string & process_name, const std::string & sr_channel_name);

};

} // namespace lib
} // namespace mrrocpp

#endif /* SR_ECP_H_ */
