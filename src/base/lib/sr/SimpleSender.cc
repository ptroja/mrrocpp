/*!
 * @file ThreadedSender.cc
 * @brief Threaded system reporting sender - definitions.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include "base/lib/sr/SimpleSender.h"

namespace mrrocpp {
namespace lib {

SimpleSender::SimpleSender(const std::string & sr_name) :
	SenderBase(sr_name)
{
}

SimpleSender::~SimpleSender()
{
}

void SimpleSender::send_package(const sr_package_t& package)
{
	Send(package);
}

} // namespace lib
} // namespace mrrocpp
