#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/festival/generator/ecp_g_festival.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace generator {

generator::generator(common::task::task& _ecp_task) :
		common::generator::generator (_ecp_task)
{
	host = ecp_t.config.value<std::string>("server_host");
	portnum = ecp_t.config.value<int>("server_port");
	test_mode = ecp_t.config.value<int>("test_mode");
	voice = "";
	sock = -1;
}

char * generator::set_phrase(const char *text)
{
	return strncpy(phrase, text, sizeof(phrase));
}

bool generator::set_voice(VOICE voice_id)
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

bool generator::first_step ( )
{
	std::string command;

	command += voice;
	command += FESTIVAL_SAY_STRING_PREFIX;
	command += phrase;
	command += FESTIVAL_SAY_STRING_SUFFIX;

	if (test_mode) {
		printf("festival_command->%s:%d = %s", host.c_str(), portnum, command.c_str());
		return true;
	}

	struct sockaddr_in server;
	struct hostent* entp;

	server.sin_family = PF_INET;

	/*
	 * this is okay to do, because gethostbyname(3) does no lookup if the 
	 * 'host' * arg is already an IP addr
	 */
	if((entp = gethostbyname(host.c_str())) == NULL) {
		fprintf(stderr, "festival_generator::first_step(): \"%s\" is unknown host; "
		        "can't connect to Festival\n", host.c_str());
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

	ssize_t written = write(sock, (const void *) command.c_str(), command.length());
	if (written == -1) {
		perror("festival_generator::first_step(): write()");
		close(sock);
		return false;
	} else if (written < (int) command.length()) {
		fprintf(stderr, "festival_generator::first_step(): write() %d of %d bytes written\n",
		        written, command.length());
		return false;
	}

	numread = 0;
	read_pending_status = 2 + (strlen(voice.c_str()) ? 1 : 0); // number of festival commands requested

	return true;
}

bool generator::next_step ( )
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
		if (numread < (int) strlen(FESTIVAL_CODE_OK)) {
			int numthisread;

			if ((numthisread = read(sock, buf+numread, sizeof(buf)-numread)) == -1) {
				perror("festival_generator::next_step(): read()");
			}

			numread += numthisread;
			buf[numread] = 0;
		}

		if (numread < (int) strlen(FESTIVAL_CODE_OK)) {
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

