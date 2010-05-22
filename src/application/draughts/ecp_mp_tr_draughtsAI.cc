#include <netdb.h>
#include <sys/socket.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <string.h>

#include "lib/exception.h"
#include <boost/exception/errinfo_api_function.hpp>
#include <boost/exception/errinfo_errno.hpp>

#include "ecp_mp_tr_draughtsAI.h"

using namespace std;

namespace mrrocpp {
namespace ecp_mp {
namespace transmitter {

TRDraughtsAI::TRDraughtsAI(TRANSMITTER_ENUM _transmitter_name, const char* _section_name, task::task& _ecp_mp_object):
	transmitter (_transmitter_name, _section_name, _ecp_mp_object){
}

TRDraughtsAI::~TRDraughtsAI(){

}

void TRDraughtsAI::AIconnect(const char * host, uint16_t serverPort){
	int socketDesc;
	struct sockaddr_in serverAddress;
	struct hostent *hostInfo;
	char c;

	hostInfo=gethostbyname(host);
	if (hostInfo==NULL) {
		cerr<<"problem interpreting host: "<<host<<"\n";
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				lib::exception::h_errno_code(h_errno) <<
				boost::errinfo_api_function("gethostbyname")
		);
	}
	socketDesc = socket(AF_INET, SOCK_STREAM, 0);
	if (socketDesc < 0) {
		int e = errno;
		cerr << "cannot create socket\n";
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(e) <<
				boost::errinfo_api_function("socket")
		);
	}

	serverAddress.sin_family = hostInfo->h_addrtype;
	std::memcpy((char *) &serverAddress.sin_addr.s_addr,hostInfo->h_addr_list[0], hostInfo->h_length);
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

	socketDescriptor=socketDesc;
}

void TRDraughtsAI::AIdisconnect(){
	close(socketDescriptor);
}

bool TRDraughtsAI::t_read(){
	if (recv(socketDescriptor, &from_va.draughts_ai, sizeof(from_va.draughts_ai), 0) < 0) {
		int e = errno;
		cerr << "didn't get response from server?";
		close(socketDescriptor);
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(e) <<
				boost::errinfo_api_function("recv")
		);
	}

	return true;
}

bool TRDraughtsAI::t_write(){
	if (send(socketDescriptor, &to_va.draughts_ai, sizeof(to_va.draughts_ai), 0) < 0) {
		int e = errno;
		cerr << "cannot send data ";
		close(socketDescriptor);
		BOOST_THROW_EXCEPTION(
				lib::exception::System_error() <<
				boost::errinfo_errno(e) <<
				boost::errinfo_api_function("send")
		);
	}

	return true;
}

} // namespace transmitter
} // namespace ecp_mp
} // namespace mrrocpp
