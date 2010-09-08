/**
 * @file
 * @brief File containing definitions of low-level (hardware access) functions.
 *
 * @author B.Bielawski
 * @author T.Wlostowski
 * @author T.Adamczyk
 * @author tkornuta
 * @date 16.06.2008
 *
 * @ingroup PCBIRD_SENSOR
 */


#include <cstdio>
#include <cstdlib>
#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cerrno>
#include <netdb.h>
#include <cmath>
#include <sys/poll.h>
#include <netinet/in.h>

#include "sensor/pcbird/birdclient.h"

namespace mrrocpp {
namespace ecp_mp {
namespace sensor {

#define MAX_RANGE ( 36.0 * 0.0254 )

#define SVCMD_POS 1
#define SVCMD_START_STREAMING 2
#define SVCMD_STOP_STREAMING 3
#define SVCMD_END_SESSION 4


int pcbird_connect(const char *addr, unsigned short port)
{
	int s, len;
	struct sockaddr_in remote;
	struct hostent *he;

	he = gethostbyname(addr);

	if (!he)
		return -1;

	if ((s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
		return -1;

	remote.sin_family = AF_INET;
	remote.sin_port = htons(port);
	remote.sin_addr.s_addr = *((unsigned long*) he->h_addr);

	len = sizeof(struct sockaddr_in);

	if (connect(s, (struct sockaddr *) &remote, len) == -1)
		return -1;

	fprintf(stderr, "pcbird_connect(): fd = %d\n", s);
	return s;
}

void pcbird_disconnect(int fd)
{
	uint8_t b = SVCMD_END_SESSION;
	send(fd, &b, 1, 0);
	close(fd);
}

int pcbird_start_streaming(int fd)
{
	uint8_t b = SVCMD_START_STREAMING;
	return send(fd, &b, 1, 0) == 1 ? 0 : -1;
}

int pcbird_stop_streaming(int fd)
{
	uint8_t b = SVCMD_STOP_STREAMING;
	return send(fd, &b, 1, 0) == 1 ? 0 : -1;
}

void decode_packet(uint8_t *payload, pcbird_pos_t *p)
{
	short xp, yp, zp, ap, bp, gp;

	xp = ntohs(*(short *) &payload[0]);
	yp = ntohs(*(short *) &payload[2]);
	zp = ntohs(*(short *) &payload[4]);

	ap = ntohs(*(short *) &payload[6]);
	bp = ntohs(*(short *) &payload[8]);
	gp = ntohs(*(short *) &payload[10]);

	p->x = ((float) xp) * MAX_RANGE / 32768.0;
	p->y = ((float) yp) * MAX_RANGE / 32768.0;
	p->z = ((float) zp) * MAX_RANGE / 32768.0;

	p->a = ((float) ap) * 180.0 / 32768.0;
	p->b = ((float) bp) * 180.0 / 32768.0;
	p->g = ((float) gp) * 180.0 / 32768.0;

	p->distance = sqrt(p->x * p->x + p->y * p->y + p->z * p->z);

	p->ts_sec = ntohl(*(int *) &payload[12]);
	p->ts_usec = ntohl(*(int *) &payload[16]);

}

int pcbird_get_single_position(int fd, pcbird_pos_t *p)
{
	uint8_t b = SVCMD_POS;
	uint8_t rxbuf[20];

	if (send(fd, &b, 1, 0) != 1)
		return -1;
	if (recv(fd, rxbuf, 20, 0) != 20)
		return -1;

	decode_packet(rxbuf, p);

	return 0;
}

int pcbird_data_avail(int fd)
{
	struct pollfd pfd;

	pfd.fd = fd;
	pfd.events = POLLIN;
	pfd.revents = 0;

	poll(&pfd, 1, 0);

	if (pfd.revents)
		return 1;

	return 0;
}

int pcbird_get_streaming_position(int fd, pcbird_pos_t *p)
{
	uint8_t rxbuf[20];

	if (recv(fd, rxbuf, 20, 0) != 20)
		return -1;
	decode_packet(rxbuf, p);

	return 0;
}


} // namespace sensor
} // namespace ecp_mp
} // namespace mrrocpp
