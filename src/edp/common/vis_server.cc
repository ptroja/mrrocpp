#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>

#include "lib/impconst.h"
#include "edp/common/edp_e_manip_and_conv.h"
#include "edp/common/vis_server.h"

#define MAXBUFLEN 100

namespace mrrocpp {
namespace edp {
namespace common {


vis_server::vis_server(manip_and_conv_effector &_master) :
	edp_extension_thread(_master), master (_master)
{}

vis_server::~vis_server()
{}


void * edp_extension_thread::thread_start(void* arg)
{
	return static_cast<vis_server*> (arg)->thread_main_loop(arg);
}

void * vis_server::thread_main_loop(void * arg)
{
	int sockfd;
	struct sockaddr_in my_addr;	// my address information
	struct sockaddr_in their_addr; // connector's address information
	socklen_t addr_len;

	uint16_t port = master.config.return_int_value("visual_udp_port");
	if (port == 0)
	{
		master.msg->message("visualisation_thread: bad or missing <visual_udp_port> config entry");
		return NULL;
	}

	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("socket()");
		exit(1);
	}

	my_addr.sin_family = AF_INET;		 // host byte order
	my_addr.sin_port = htons(port);	 // short, network byte order
	my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
	memset(my_addr.sin_zero, '\0', sizeof my_addr.sin_zero);

	if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof my_addr) == -1)
	{
		perror("bind()");
		exit(1);
	}

	addr_len = sizeof their_addr;

	while (1)
	{
		int numbytes;
		char buf[MAXBUFLEN];

		if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
				(struct sockaddr *)&their_addr, &addr_len)) == -1)
		{
			perror("recvfrom()");
			break;
		}

		//	printf("got %d bytes packet from %s\n", numbytes, inet_ntoa(their_addr.sin_addr));

		double tmp[master.number_of_servos];
		master.master_joints_read(tmp);

		// korekta aby polozenia byly wzgledem poprzedniego czlonu
		switch (master.robot_name)
		{
			case lib::ROBOT_IRP6_ON_TRACK:
				tmp[3] -= tmp[2] +M_PI_2;
				tmp[4] -= tmp[3] + tmp[2] +M_PI_2;
				break;
			case lib::ROBOT_IRP6_POSTUMENT:
				tmp[2] -= tmp[1] +M_PI_2;
				tmp[3] -= tmp[2] + tmp[1] +M_PI_2;
				break;
			default:
				break;
		}

		struct
		{
			int synchronised;
			float joints[MAX_SERVOS_NR];
		}
		reply;

		reply.synchronised = (master.is_synchronised()) ? 1 : 0;

		for (int i = 0; i < master.number_of_servos; i++)
		{
			reply.joints[i] = static_cast <float> (tmp[i]);
		}

		numbytes = sendto(sockfd, &reply, sizeof(reply),
				0, (struct sockaddr *) &their_addr, addr_len);
		if (numbytes == -1)
		{
			perror("sendto()");
			break;
		}
		else if (numbytes < (ssize_t) sizeof(reply))
		{
			fprintf(stderr, "send only %d of %d bytes\n", numbytes, sizeof(reply));
			break;
		}

	}

	close(sockfd);

	return NULL;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

