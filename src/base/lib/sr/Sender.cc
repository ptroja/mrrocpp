/*!
 * @file Sender.cc
 * @brief System reporting sender base class - definitions.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include <boost/thread/thread.hpp>

#include "base/lib/messip/messip_dataport.h"

#include "base/lib/sr/Sender.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

Sender::Sender(const std::string & sr_name)
{
	unsigned int tmp = 0;
	while ((ch = messip::port_connect(sr_name)) == NULL) {
		if (tmp++ < lib::CONNECT_RETRY) {
			boost::this_thread::sleep(lib::CONNECT_DELAY);
		} else {
			fprintf(stderr, "messip::port_connect(\"%s\") @ %s:%d: %s\n",
					sr_name.c_str(), __FILE__, __LINE__, strerror(errno));
			// TODO: throw
			assert(0);
		}
	}

	assert(ch);
}

Sender::~Sender() {
	if(messip::port_disconnect(ch) == -1) {
		perror("messip::port_disconnect()");
	}
}

void Sender::send_package(const sr_package_t& package)
{
	// TODO: error check and throw an exception
	int status = messip::port_send_async(ch, 0, 0, package, 0);

	if(status == MESSIP_MSG_TIMEOUT) {
		std::cerr << "SR: send would block, aborted" << std::endl;
	} else if (status < 0) {
		std::cerr << "SR: send failed" << std::endl;
	}
}

} // namespace lib
} // namespace mrrocpp
