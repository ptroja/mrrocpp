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

	while(true) {
		int32_t type, subtype;
		std::string config_file;

		const int rcvid = messip::port_receive(ch,
				type, subtype,
				config_file);

		if (rcvid < 0) {
			continue;
		}

		std::cout << "configsrv_m: rcvid " << rcvid <<
				" : " << config_file <<
				" lenght " << config_file.length() <<
				std::endl;

		if(config_file.length() > 0) {
			// TODO: try/catch -> ack/nack ?
			config.change_ini_file(config_file);
		}

		property_trees_t ptrees;
		ptrees.common_file_pt = config.common_file_pt;
		ptrees.file_pt = config.file_pt;

		try {
			messip::port_reply(ch, rcvid, 0, ptrees);
		} catch (std::exception & e) {
			std::cerr << "configsrv exception: " << e.what() << std::endl;
			break;
		}
	}

	messip::port_delete(ch);

	return 0;
}
