/*!
 * @file SenderBase.cc
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

#include "base/lib/messip/messip_dataport.h"

#include "base/lib/sr/SenderBase.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

SenderBase::SenderBase(const std::string & sr_name)
{
	unsigned int tmp = 0;
	while ((ch = messip::port_connect(sr_name)) == NULL) {
		if (tmp++ < lib::CONNECT_RETRY) {
			usleep(lib::CONNECT_DELAY);
		} else {
			fprintf(stderr, "messip::port_connect(\"%s\") @ %s:%d: %s\n",
					sr_name.c_str(), __FILE__, __LINE__, strerror(errno));
			// TODO: throw
			assert(0);
		}
	}

	assert(ch);
}

SenderBase::~SenderBase() {
	if(messip::port_disconnect(ch) == -1) {
		perror("messip::port_disconnect()");
	}
}

void SenderBase::Send(const sr_package_t & sr_mess)
{
	// TODO: error check and throw an exception
	messip::port_send_async(ch, 0, 0, sr_mess);
}

} // namespace lib
} // namespace mrrocpp
