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

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#else
#include <unistd.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#endif

#include "base/lib/sr/SenderBase.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {

#if !defined(USE_MESSIP_SRR)
SenderBase::SenderBase(const std::string & sr_name)
{
	// kilka sekund (~1) na otworzenie urzadzenia
	unsigned int tmp = 0;
	while ((fd = name_open(sr_name.c_str(), NAME_FLAG_ATTACH_GLOBAL)) < 0) {
		if ((tmp++) < lib::CONNECT_RETRY) {
			delay(lib::CONNECT_DELAY);
		} else {
			// TODO: throw
			std::perror("SR cannot be located ");
			throw;
		}
	}
}

SenderBase::~SenderBase()
{
	name_close(fd);
}

void SenderBase::Send(const sr_package_t & package)
{
	int16_t status;

	// TODO: error check and throw an exception
	MsgSend(fd, &package, sizeof(package), &status, sizeof(status));
}
#else /* USE_MESSIP_SRR */
SenderBase::SenderBase(const std::string & sr_name)
{
	unsigned int tmp = 0;
	while ((ch = messip::port_connect(sr_name)) == NULL) {
		if (tmp++ < 50) {
			delay(50);
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
	messip::port_send_sync(ch, 0, 0, sr_mess);
}
#endif /* !USE_MESSIP_SRR */

} // namespace lib
} // namespace mrrocpp
