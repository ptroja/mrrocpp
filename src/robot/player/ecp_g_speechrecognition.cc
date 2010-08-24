#include <unistd.h>

#include "base/ecp/player/generator/ecp_g_speechrecognition.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace generator {

speechrecognition::speechrecognition(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	std::string hostname = ecp_t.config.value <std::string> ("player_hostname");
	client = new PlayerClient(hostname.c_str(), PLAYER_PORTNUM);

	int device_index = ecp_t.config.value <int> ("device_index");
	device = new SpeechRecognitionProxy(client, device_index, 'r');
}

speechrecognition::~speechrecognition()
{
	delete device;
	delete client;
}

bool speechrecognition::first_step()
{
	if (device->fresh) {
		device->Clear();
		device->fresh = false;
	}
	return true;
}

bool speechrecognition::next_step()
{
#if 1
	// do not block
	if (client->Peek(0)) {
		client->Read();
	}
	usleep(20000);
#else
	// block
	client->Read();
#endif

	if (device->fresh) {
		device->Clear();
		device->fresh = false;
		strncpy(ecp_t.ecp_reply.ecp_2_mp_string, device->rawText, ECP_2_MP_STRING_SIZE);
		return false;
	}

	return true;
}

}
} // namespace player
} // namespace ecp
} // namespace mrrocpp


