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

#include "base/lib/messip/messip_dataport.h"

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
	configsrv config(argv[1], argv[2]);

	messip_channel_t *ch = messip::port_create(CONFIGSRV_CHANNEL_NAME);
	assert(ch);

	if (signal(SIGINT, sigint_handler) == SIG_ERR) {
		perror("signal()");
	}

	try {
		while(true) {
			int32_t type, subtype;
			config_query_t query;

			const int rcvid = messip::port_receive(ch,
					type, subtype,
					query);

			if (rcvid < 0) {
				continue;
			}

			config_query_t reply;
			reply.flag = false;

			if(query.flag) {
				//! Flag set means request to change a config file
				try {
					config.change_ini_file(query.key);
					reply.flag = true;
				} catch (std::exception & e) {
					// Do nothing. Reply will send the error flag.
				}
			} else {
				//! Flag not set means request about config value
				try {
					reply.key = config.value(query.key);
					reply.flag = true;
				} catch (boost::property_tree::ptree_error & e) {
					// Do nothing. Reply will send the error flag.
				}
			}
			messip::port_reply(ch, rcvid, 0, reply);
		}
	} catch (std::exception & e) {
		std::cerr << "configsrv exception: port_reply failed: " << e.what() << std::endl;
	}

	messip::port_delete(ch);

	return 0;
}
