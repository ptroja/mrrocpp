// -------------------------------------------------------------------------
// Proces: 	EFFECTOR CONTROL PROCESS (lib::ECP)
// Plik:			ecp_mp_sensor.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Definicja konstruktora bazowej klasy czujnikow po stronie procesu ECP - jeden dla wszystkich.
// Autor:		yoyek/tkornuta
// Data:		10.11.2005
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"				// numery bledow

#include "base/lib/configurator.h"
#include "base/ecp_mp/transmitter.h"
#include "base/ecp_mp/ecp_mp_task.h"
#include "ecp_mp_tr_rc_windows.h"

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

rc_win_buf_typedef *rc_windows::rc_win_buf = NULL;

rc_windows::rc_windows(lib::TRANSMITTER_t _transmitter_name, const char* _section_name, task::task& _ecp_mp_object) :
	rc_windows_transmitter_t(_transmitter_name, _section_name, _ecp_mp_object)
{
	if (!rc_win_buf) {
		rc_win_buf = new rc_win_buf_typedef;
	} else {
		printf("powolano juz obiekt klasy transmitter\n");
	}

	rc_win_buf->solver_hostname = _ecp_mp_object.config.value <std::string> ("solver_hostname", _section_name).c_str();
	rc_win_buf->solver_port = _ecp_mp_object.config.value <int> ("solver_port", _section_name);
}

rc_windows::~rc_windows()
{
	delete rc_win_buf;
}

void * rc_windows::do_query(void * arg)
{
	boost::mutex::scoped_lock lock(rc_win_buf->mtx);

	int sock;
	fd_set fds;
	struct timeval timeout;
	int retval;

	/*
	 switch (to_va.rc_windows.rc_state[i]) {
	 case 'Y':
	 case 'y':
	 pattern[i] = 'y'; break;
	 case 'W':
	 case 'w':
	 pattern[i] = 'w'; break;
	 case 'R':
	 case 'r':
	 pattern[i] = 'r'; break;
	 case 'O':
	 case 'o':
	 pattern[i] = 'o'; break;
	 case 'B':
	 case 'b':
	 pattern[i] = 'b'; break;
	 case 'G':
	 case 'g':
	 pattern[i] = 'g'; break;
	 }
	 */

	sock = make_socket(rc_win_buf->solver_hostname, rc_win_buf->solver_port);

	//  int l = strlen(rc_win_buf->request);
	if (write(sock, rc_win_buf->request, strlen(rc_win_buf->request)) != (ssize_t) strlen(rc_win_buf->request)) {
		perror("write()");
		close(sock);
		return NULL;
	}

	/* Initialize the file descriptor set. */
	FD_ZERO(&fds);
	FD_SET(sock, &fds);

	/* Initialize the timeout data structure. */
	timeout.tv_sec = 60 * 2;
	timeout.tv_usec = 0;

	/* `select' returns 0 if timeout, 1 if input available, -1 if error. */
	retval = select(FD_SETSIZE, &fds, NULL, NULL, &timeout);
	if (retval == -1) {
		perror("select()");
		close(sock);
		return NULL;
	} else if (retval == 0) {
		fprintf(stderr, "socket timeout\n");
		close(sock);
		return NULL;
	}

	retval = read(sock, rc_win_buf->response, 1024);
	if (retval < 0) {
		perror("read()");
		close(sock);
		return NULL;
	}

	rc_win_buf->response[retval] = '\0';

	printf("%s", rc_win_buf->response);

	close(sock);

	return NULL;
}

int rc_windows::make_socket(const std::string & hostname, uint16_t port)
{
	int sock;
	struct sockaddr_in server;
	struct hostent *hostinfo;

	/* Create the socket. */
	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket");
		return -1;
	}

	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	hostinfo = gethostbyname(hostname.c_str());
	if (hostinfo == NULL) {
		std::cerr << "Unknown host: " << hostname << std::endl;
		return -1;
	}
	server.sin_addr = *(struct in_addr *) hostinfo->h_addr;

	if (connect(sock, (struct sockaddr*) &server, sizeof(server)) == -1) {
		perror("connect");
		return -1;
	}

	return sock;
}

bool rc_windows::t_write()
{

	snprintf(rc_win_buf->request, sizeof(rc_win_buf->request), "GET /?%s HTTP/1.0\r\n", to_va.rc_state);

	pthread_create(&worker, NULL, do_query, NULL);

	return true;
}

bool rc_windows::t_read(bool wait)
{

	if (wait) {

		boost::mutex::scoped_lock lock(rc_win_buf->mtx);

		printf("W SEMAFORZE 1 %zd\n", strlen(rc_win_buf->response) - 33);

		int l = strlen(rc_win_buf->response) - 33 - 16;
		if (l < 0)
			l = 0;

		strncpy(from_va.sequence, rc_win_buf->response + 33, l);
		from_va.sequence[l] = '\0';

	} else {

		printf("W SEMAFORZE2 %zd\n", strlen(rc_win_buf->response) - 33);
		int l = strlen(rc_win_buf->response) - 33 - 16;
		if (l < 0)
			l = 0;
		strncpy(from_va.sequence, rc_win_buf->response + 33, l);
		from_va.sequence[l] = '\0';

	}
	return true;
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp

