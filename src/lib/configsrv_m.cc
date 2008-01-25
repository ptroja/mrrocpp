#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "lib/configsrv.h"
#include "messip/messip.h"
#include "lib/config_types.h"

int
main(int argc, char *argv[])
{
	configsrv *config = new configsrv(argv[1], argv[2], argv[3]);

	messip_channel_t *ch = messip_channel_create(NULL, CONFIGSRV_CHANNEL_NAME, MESSIP_NOTIMEOUT, 0);
	assert(ch);

	while(1) {
		int rcvid;
		int32_t type, subtype;
		config_request req;	
		config_msg_t config_msg;

		rcvid = messip_receive(ch,
				&type, &subtype,
				&config_msg, sizeof(config_msg),
				MESSIP_NOTIMEOUT);

		if (rcvid < 0) {
			continue;
		}

		req = (config_request) type;

		switch(req) {
			case CONFIG_CHANGE_INI_FILE:
				{
					config->change_ini_file(config_msg.configfile);
					messip_reply(ch, rcvid,
						0, NULL, 0,
						MESSIP_NOTIMEOUT);
				}
				break;
			case CONFIG_RETURN_INT_VALUE:
				{
					const int rep = config->return_int_value(
							config_msg.key,
							config_msg.section);

					messip_reply(ch, rcvid,
						0, &rep, sizeof(rep),
						MESSIP_NOTIMEOUT);
				}
				break;
			case CONFIG_RETURN_DOUBLE_VALUE:
				{
					const double rep = config->return_double_value(
							config_msg.key,
							config_msg.section);

					messip_reply(ch, rcvid,
						0, &rep, sizeof(rep),
						MESSIP_NOTIMEOUT);
				}
				break;
			case CONFIG_RETURN_STRING_VALUE:
				{
					const char *rep = config->return_string_value(
							config_msg.key,
							config_msg.section);

					//printf("convigsrv(CONFIG_RETURN_STRING_VALUE, key:%s, section:%s) = %s\n",
					//		config_msg.key, config_msg.section, rep);

					messip_reply(ch, rcvid,
						0, rep, strlen(rep)+1,
						MESSIP_NOTIMEOUT);
					delete[] rep;
				}
				break;
			case CONFIG_EXISTS:
				{
					const bool rep = config->exists(
							config_msg.key,
							config_msg.section);

					messip_reply(ch, rcvid,
						0, &rep, sizeof(rep),
						MESSIP_NOTIMEOUT);
				}
				break;
			default:
				messip_reply(ch, rcvid,
						-1, (void *) NULL, 0,
						MESSIP_NOTIMEOUT);
		}
	}

	messip_channel_delete(ch, MESSIP_NOTIMEOUT);

	return 0;
}
