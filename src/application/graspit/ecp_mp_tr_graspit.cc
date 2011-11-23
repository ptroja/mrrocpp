#include <netdb.h>
#include <sys/socket.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>

#include "ecp_mp_tr_graspit.h"
#include "../../base/ecp_mp/ecp_mp_exceptions.h"

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

TRGraspit::TRGraspit(lib::TRANSMITTER_t _transmitter_name, const char* _section_name, task::task& _ecp_mp_object) :
		GraspitTransmitter_t(_transmitter_name, _section_name, _ecp_mp_object), socketDescriptor(-1)
{
}

TRGraspit::~TRGraspit()
{
}

void TRGraspit::TRconnect(const char *host, unsigned short int serverPort)
{
	struct hostent * hostInfo = gethostbyname(host);
	if (hostInfo == NULL) {
		cerr << "problem interpreting host: " << host << "\n";
		BOOST_THROW_EXCEPTION(exception::se_tr());
	}

	cout << "host ok\n";

	int socketDesc = socket(AF_INET, SOCK_STREAM, 0);
	if (socketDesc == -1) {
		cerr << "cannot create socket\n";
		BOOST_THROW_EXCEPTION(exception::se_tr());
	}

	cout << "socket ok\n";

	struct sockaddr_in serverAddress;
	serverAddress.sin_family = hostInfo->h_addrtype;
	std::memcpy((char *) &serverAddress.sin_addr.s_addr, hostInfo->h_addr_list[0], hostInfo->h_length);
	serverAddress.sin_port = htons(serverPort);

	if (connect(socketDesc, (struct sockaddr *) &serverAddress, sizeof(serverAddress)) < 0) {
		cerr << "cannot connect\n";
		BOOST_THROW_EXCEPTION(exception::se_tr());
	}

	cout << "connect ok\n";

	socketDescriptor = socketDesc;
}

void TRGraspit::TRdisconnect()
{
	if (socketDescriptor != -1) {
		close(socketDescriptor);
	}
}

bool TRGraspit::t_read()
{
	if (recv(socketDescriptor, &from_va, sizeof(from_va), 0) < 0) {
		cerr << "didn't get response from server?";
		close(socketDescriptor);
		BOOST_THROW_EXCEPTION(exception::se_tr());
	}

	cout << "read ok\n";

	return true;
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp
