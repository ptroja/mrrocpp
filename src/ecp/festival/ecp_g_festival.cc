#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/festival/ecp_g_festival.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace generator {

festival_generator::festival_generator(common::task::base& _ecp_task) :
		base (_ecp_task)
{
	host = ecp_t.config.return_string_value("server_host");
	portnum = ecp_t.config.return_int_value("server_port");
	test_mode = ecp_t.config.return_int_value("test_mode");
	voice = "";
	sock = -1;
}

festival_generator::~festival_generator()
{
	delete [] host;
}

char * festival_generator::set_phrase(const char *text)
{
	return strncpy(phrase, text, sizeof(phrase));
}

bool festival_generator::set_voice(VOICE voice_id)
{
	switch (voice_id) {
		case CURRENT_VOICE:
			break;
		case POLISH_VOICE:
			voice = FESTIVAL_POLISH_VOICE;
			break;
		case ENGLISH_VOICE:
			voice = FESTIVAL_ENGLISH_VOICE;
			break;
		default:
			return false;
	}
	
	return true;			
}

bool festival_generator::first_step ( )
{
	int command_max_len = strlen(voice)
		+strlen(FESTIVAL_SAY_STRING_PREFIX)+sizeof(phrase)+strlen(FESTIVAL_SAY_STRING_SUFFIX)
		+1;
	char command[command_max_len];

	snprintf(command, command_max_len, "%s%s%s%s",
			 voice,
	         FESTIVAL_SAY_STRING_PREFIX,
	         this->phrase,
	         FESTIVAL_SAY_STRING_SUFFIX);

	int command_len = strlen(command);

	if (test_mode) {
		printf("festival_command->%s:%d = %s", host, portnum, command);
		return true;
	}

	struct sockaddr_in server;
	struct hostent* entp;

	server.sin_family = PF_INET;

	/*
	 * this is okay to do, because gethostbyname(3) does no lookup if the 
	 * 'host' * arg is already an IP addr
	 */
	if((entp = gethostbyname(host)) == NULL) {
		fprintf(stderr, "festival_generator::first_step(): \"%s\" is unknown host; "
		        "can't connect to Festival\n", host);
		return false;
	}

	memcpy(&server.sin_addr, entp->h_addr_list[0], entp->h_length);

	server.sin_port = htons(portnum);
	
	if (sock >=0) {
		if (close(sock) == -1) {
			perror("festival_generator::first_step(): close()");
		}
	}

	/* make a new socket */
	if((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		perror("festival_generator::first_step(): socket()");
		return false;
	}

	/* hook it up */
	if(connect(sock, (struct sockaddr*)&server, sizeof(server)) == -1) {
		perror("festival_generator::first_step(): connect()");
		return false;
	}

	/* make it nonblocking */
	if(fcntl(sock,F_SETFL,O_NONBLOCK) < 0) {
		perror("festival_generator::first_step(): fcntl()");
		return false;
	}

	int written = write(sock, (const void *) command, command_len);
	if (written == -1) {
		perror("festival_generator::first_step(): write()");
		close(sock);
		return false;
	} else if (written < command_len) {
		fprintf(stderr, "festival_generator::first_step(): write() %d of %d bytes written\n",
		        written, command_len);
		return false;
	}

	numread = 0;
	read_pending_status = 2 + (strlen(voice) ? 1 : 0); // number of festival commands requested

	return true;
}

bool festival_generator::next_step ( )
{
	if(test_mode) {
		return false;
	}

	if (!read_pending_status) {
		if (close(sock) == -1) {
			perror("festival_generator::first_step(): close()");
		}
		sock = -1;
		return false;
	}

	fd_set rd;

	FD_SET(sock, &rd);

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	bool has_data = false;

	/* read the resultant string back */
	switch (select(sock + 1, &rd, NULL, NULL, &timeout)) {
		case -1:
			perror("festival_generator::next_step(): select()");
			break;
		case 0:
			/* timeout */
			break;
		default:
			has_data = (FD_ISSET(sock, &rd)) ? true : false;
			break;
	}

	if (has_data) {
		if (numread < strlen(FESTIVAL_CODE_OK)) {
			int numthisread;

			if ((numthisread = read(sock, buf+numread, sizeof(buf)-numread)) == -1) {
				perror("festival_generator::next_step(): read()");
			}

			numread += numthisread;
			buf[numread] = 0;
		}

		if (numread < strlen(FESTIVAL_CODE_OK)) {
			fprintf(stderr, "festival_generator::next_step(): something went wrong, "
			        "expected %d bytes of code, but got %d\n",
			        (int) strlen(FESTIVAL_CODE_OK),numread);
			close(sock);
			sock = -1;
			return false;
		}

		char *ptr = buf;
		while (ptr && *ptr) {
			if (!strncmp(ptr,FESTIVAL_CODE_OK, strlen(FESTIVAL_CODE_OK))) {
				/* command OK */
				read_pending_status--;
			} else if (!strncmp(ptr, FESTIVAL_CODE_ERR, strlen(FESTIVAL_CODE_ERR))) {
				/* something went wrong */
				read_pending_status--;
			}
			ptr = strchr(ptr, '\n');
			if (ptr) {
				ptr++;
			}
		}

		numread = 0;
	}

	usleep(20000);

	return true;
}

}
} // namespace festival
} // namespace ecp
} // namespace mrrocpp

