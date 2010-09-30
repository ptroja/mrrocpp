/*!
 * @file configsrv.h
 * @brief Configuration server
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <cstdio>
#include <stdint.h>
#include <cassert>
#include <cstring>
#include <csignal>
#include <cstdlib>
#include <string>

#if defined(USE_MESSIP_SRR)
#include "base/lib/messip/messip_dataport.h"
#endif

#include "base/lib/configsrv.h"
#include "base/lib/config_types.h"

void
sigint_handler(int signum)
{
	exit(-1);
}

int
main(int argc, char *argv[])
{
	configsrv config(argv[1], argv[2], argv[3]);

	messip_channel_t *ch = messip::port_create(CONFIGSRV_CHANNEL_NAME);
	assert(ch);

	if (signal(SIGINT, sigint_handler) == SIG_ERR) {
		perror("signal()");
	}

	while(true) {
		int32_t type, subtype;
		config_msg_t config_msg;

		int rcvid = messip::port_receive(ch,
				type, subtype,
				config_msg);

		if (rcvid < 0) {
			continue;
		}

		config_request_t req = (config_request_t) type;

		switch(req) {
			case CONFIG_CHANGE_INI_FILE:
				{
					config.change_ini_file(config_msg.configfile);
					messip::port_reply_ack(ch, rcvid);
				}
				break;
			case CONFIG_RETURN_INT_VALUE:
				{
					const int reply = config.value<int>(
							config_msg.query.key,
							config_msg.query.section);

					messip::port_reply(ch, rcvid, 0, reply);
				}
				break;
			case CONFIG_RETURN_DOUBLE_VALUE:
				{
					const double reply = config.value<double>(
							config_msg.query.key,
							config_msg.query.section);

					messip::port_reply(ch, rcvid, 0, reply);
				}
				break;
			case CONFIG_RETURN_STRING_VALUE:
				{
					const std::string reply = config.value<std::string>(
							config_msg.query.key,
							config_msg.query.section);

					messip_reply(ch, rcvid,
						0, reply.c_str(), reply.size()+1,
						MESSIP_NOTIMEOUT);
				}
				break;
			case CONFIG_EXISTS:
				{
					const bool reply = config.exists(
							config_msg.query.key,
							config_msg.query.section);

					messip::port_reply(ch, rcvid, 0, reply);
				}
				break;
			default:
				messip::port_reply_nack(ch, rcvid);
		}
	}

	messip::port_delete(ch);

	return 0;
}
