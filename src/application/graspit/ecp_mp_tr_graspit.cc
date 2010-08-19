#include <netdb.h>
#include <sys/socket.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>

#include "lib/exception.h"
#include <boost/exception/errinfo_api_function.hpp>
#include <boost/exception/errinfo_errno.hpp>

#include "ecp_mp_tr_graspit.h"

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

TRGraspit::TRGraspit(TRANSMITTER_ENUM _transmitter_name, const std::string & _section_name, task::task& _ecp_mp_object):
	GraspitTransmitter_t (_transmitter_name, _section_name, _ecp_mp_object){
}

TRGraspit::~TRGraspit()
{
}

void TRGraspit::TRconnect(const char *host, uint16_t serverPort){
	struct hostent *hostInfo;

	hostInfo=gethostbyname(host);
	if (hostInfo==NULL) {
		cerr<<"problem interpreting host: "<<host<<"\n";
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				lib::exception::h_errno_code(h_errno) <<
				boost::errinfo_api_function("gethostbyname")
		);
	}

	cout << "host ok\n";

	int socketDesc = socket(AF_INET, SOCK_STREAM, 0);
	if (socketDesc < 0) {
		int e = errno;
		cerr << "cannot create socket\n";
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(e) <<
				boost::errinfo_api_function("socket")
		);
	}

	cout << "socket ok\n";

	struct sockaddr_in serverAddress;
	serverAddress.sin_family = hostInfo->h_addrtype;
	std::memcpy((char *) &serverAddress.sin_addr.s_addr, hostInfo->h_addr_list[0], hostInfo->h_length);
	serverAddress.sin_port = htons(serverPort);


	if (connect(socketDesc,(struct sockaddr *) &serverAddress,sizeof(serverAddress)) < 0) {
		int e = errno;
		cerr << "cannot connect\n";
		close(socketDesc);
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(e) <<
				boost::errinfo_api_function("connect")
		);
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

bool TRGraspit::t_read(){
	if (recv(socketDescriptor, &from_va, sizeof(from_va), 0) < 0) {
		int e = errno;
		cerr << "didn't get response from server?";
		close(socketDescriptor);
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(e) <<
				boost::errinfo_api_function("recv")
		);
	}

	cout << "read ok\n";

	return true;
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp
