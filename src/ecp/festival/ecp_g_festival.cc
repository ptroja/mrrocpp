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

festival_generator::festival_generator(ecp_task& _ecp_task) :
	ecp_generator (_ecp_task, true),
	read_pending(false)
{
	host = ecp_t.config->return_string_value("server_host");
	portnum = ecp_t.config->return_int_value("server_port");
}

char * festival_generator::set_phrase(const char *text)
{
	return strncpy(phrase, text, sizeof(phrase));
}

bool festival_generator::first_step ( ) 
{
	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);

	ecp_t.mp_buffer_receive_and_send ();

	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("first_step()::INVALID_MP_COMMAND = %d\n", INVALID_MP_COMMAND);
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	struct sockaddr_in server;
	struct hostent* entp;

	server.sin_family = PF_INET;

	/* 
	 * this is okay to do, because gethostbyname(3) does no lookup if the 
	 * 'host' * arg is already an IP addr
	 */
	if((entp = gethostbyname(host)) == NULL)
	{
		fprintf(stderr, "festival_generator::first_step(): \"%s\" is unknown host; "
			"can't connect to Festival\n", host);
		delete [] host;
		return false;
	}
	
	delete [] host;

	memcpy(&server.sin_addr, entp->h_addr_list[0], entp->h_length);

	server.sin_port = htons(portnum);

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

	int command_max_len = strlen(FESTIVAL_SAY_STRING_PREFIX)+sizeof(phrase)+strlen(FESTIVAL_SAY_STRING_SUFFIX);
	char command[command_max_len];

	snprintf(command, command_max_len, "%s%s%s", 
			FESTIVAL_SAY_STRING_PREFIX,
			this->phrase,
			FESTIVAL_SAY_STRING_SUFFIX);

	int command_len = strlen(command);

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
	read_pending = true;

	return true;
}

bool festival_generator::next_step ( ) 
{
	if (ecp_t.pulse_check()) 
	{
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	}
	else
	{
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}

	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			//the_robot->create_command ();
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	if (!read_pending) {
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
			has_data = true;
			break;
	}

	if (has_data) {
		if (numread < strlen(FESTIVAL_CODE_OK)) {
			int numthisread;

			if ((numthisread = read(sock, buf+numread, strlen(FESTIVAL_CODE_OK)-numread)) == -1) {
				perror("festival_generator::next_step(): read()");
			}

			numread += numthisread;
		}

		if (numread < strlen(FESTIVAL_CODE_OK)) {
			fprintf(stderr, "festival_generator::next_step(): something went wrong, "
					"expected %d bytes of code, but got %d\n",
					(int) strlen(FESTIVAL_CODE_OK),numread);
			close(sock);
			return false;
		}

		if (strcmp(buf,FESTIVAL_CODE_OK)) {
			/* something went wrong */
		} else {
			/* command OK */
		}

		close(sock);

		return false;
	}

	usleep(20000);

	return true;
}
